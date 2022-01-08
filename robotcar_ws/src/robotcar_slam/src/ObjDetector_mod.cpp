#include "myslam/ObjDetector_mod.h"

namespace myslam
{
    Detector::Detector()
    {
    }

    Detector::~Detector()
    {
    }

    /**
 * @brief 网络初始化
 * 
 */
    void Detector::loadNetWork()
    {
        // 加载目标标签
        ifstream ifs(Param::detector_names_.c_str());
        string line;
        while (getline(ifs, line))
            classes_.push_back(line);

        // 加载网络并设置参数
        net_ = readNetFromDarknet(Param::detector_config_, Param::detector_weight_);
        // net_.setPreferableBackend(DNN_BACKEND_OPENCV);
        net_.setPreferableBackend(DNN_BACKEND_CUDA);
        // net_.setPreferableTarget(DNN_TARGET_CPU);
        net_.setPreferableTarget(DNN_TARGET_CUDA);

        // 获取网络输出层的名字
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayersIdx = net_.getUnconnectedOutLayers();
        //get the names of all the layers in the network
        vector<String> layersNames = net_.getLayerNames();
        // Get the names of the output layers in names
        outlayers_.resize(outLayersIdx.size());
        for (size_t i = 0; i < outLayersIdx.size(); ++i)
            outlayers_[i] = layersNames[outLayersIdx[i] - 1];
    }

    /**
 * @brief 网络前向传播
 * 
 */
    void Detector::detectObject(Frame::Ptr curr)
    {
        // 图像预处理
        Mat blob;
        // blobFromImage(curr->left_, blob, 1 / 255.0, Size(inp_width_, inp_height_), Scalar(0, 0, 0), true, false);
        blobFromImage(curr->left_, blob, 1 / 255.0, Size(Param::detector_inpwidth_, Param::detector_inpheight_));

        // 网络输入
        net_.setInput(blob);

        // 网络前向传播
        // FIXME 不能处理单通道灰度图像
        vector<Mat> outs;
        net_.forward(outs, outlayers_);

        // 后处理
        postProcess(curr, outs);

        // 制作当前帧目标掩码
        makeObjMask(curr);
    }

    /**
 * @brief 筛选最大概率检测结果并进行非极大值抑制
 * 
 * @param curr 当前帧
 * @param outs 
 */
    void Detector::postProcess(Frame::Ptr curr, const vector<Mat> &outs)
    {
        vector<int> classIds;
        vector<float> confidences;
        vector<Rect> boxes;

        for (size_t i = 0; i < outs.size(); ++i)
        {
            float *data = (float *)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
                // Get the value and location of the maximum score
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > Param::detector_thr_ && classIdPoint.x < 9)
                {
                    if (classIdPoint.x == 4 || classIdPoint.x == 6 || classIdPoint.x == 8) // airplane,train,boat剔除
                        continue;
                    else if (classIdPoint.x == 3) // motocycle=bicycle
                        classIdPoint.x = 1;
                    else if (classIdPoint.x == 5 || classIdPoint.x == 7) // car=bus=truck
                        classIdPoint.x = 2;
                    int centerX = (int)(data[0] * Camera::width_);
                    int centerY = (int)(data[1] * Camera::height_);
                    int width = (int)(data[2] * Camera::width_);
                    int height = (int)(data[3] * Camera::height_);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    left = max(0, min(left, Camera::width_ - 1));
                    top = max(0, min(top, Camera::height_ - 1));
                    int right = max(0, min(left + width, Camera::width_ - 1));
                    int bot = max(0, min(top + height, Camera::height_ - 1));

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, right - left, bot - height));
                }
            }
        }

        // 非极大值抑制
        vector<int> indices;
        NMSBoxes(boxes, confidences, Param::detector_thr_, Param::detector_nmsthr_, indices);

        // 生成目标并加入到当前帧
        for (int i = 0; i < indices.size(); i++)
        {
            int id = indices[i];
            int left = boxes[id].x;
            int right = boxes[id].x + boxes[id].width;
            int top = boxes[id].y;
            int bot = boxes[id].y + boxes[id].height;
            int area = boxes[id].area();
            float score = confidences[id];

            Object::Ptr obj(new myslam::Object);
            obj->left_ = left;
            obj->right_ = right;
            obj->top_ = top;
            obj->bot_ = bot;
            obj->score_ = score;
            obj->area_ = area;
            obj->class_id_ = classIds[id];
            obj->label_ = classes_[classIds[id]];
            obj->r_ = rand() % (255 + 1);
            obj->b_ = rand() % (255 + 1);
            obj->g_ = rand() % (255 + 1);
            curr->objects_.push_back(obj);
        }
        curr->mobjs_matched_ = vector<Object::Ptr>(curr->objects_.size(), nullptr);
    }

    /**
 * @brief 制作当前帧的目标掩码
 * 
 * @param curr 当前帧
 */
    void Detector::makeObjMask(Frame::Ptr curr)
    {
        int object_id = 1;
        for (auto iter : curr->objects_)
        {
            int left = iter->left_;
            int top = iter->top_;
            int right = iter->right_;
            int bot = iter->bot_;
            curr->mask_(cv::Range(top, bot), cv::Range(left, right)) = object_id;
            object_id++;
        }
    }

} // namespace myslam