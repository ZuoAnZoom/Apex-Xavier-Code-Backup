#include "myslam/ObjSegment_mod.h"

namespace myslam
{
    Segment::Segment()
    {
    }

    Segment::~Segment()
    {
    }
    /**
 * @brief 网络初始化
 * 
 */
    void Segment::loadNetWork()
    {
        // 加载目标标签
        ifstream ifs(Param::segment_names_.c_str());
        string line;
        while (getline(ifs, line))
            classes_.push_back(line);

        // 加载网络并设置参数
        net_ = readNetFromTensorflow(Param::segment_weight_, Param::segment_config_);
        // net.setPreferableBackend(DNN_BACKEND_OPENCV);
        net_.setPreferableBackend(DNN_BACKEND_CUDA);
        // net.setPreferableTarget(DNN_TARGET_CPU);
        net_.setPreferableTarget(DNN_TARGET_CUDA);
    }

    /**
 * @brief 目标实例分割
 * 
 * @param curr 当前帧
 */
    void Segment::segmentObject(Frame::Ptr curr)
    {
        // 图像预处理
        Mat blob;
        // blobFromImage(frame, blob, 1.0, Size(frame.cols, frame.rows), Scalar(), true, false);
        blobFromImage(curr->left_, blob);

        // 网络输入
        net_.setInput(blob);

        // 网络前向传播
        vector<String> outNames(2);
        outNames[0] = "detection_out_final";
        outNames[1] = "detection_masks";
        vector<Mat> outs;
        net_.forward(outs, outNames);

        // 后处理
        postprocess(curr, outs);

        // 制作目标掩码
        maskOBjMask(curr);
    }

    /**
 * @brief 提取边界框和目标掩码，生成目标并加入到当前帧
 * 
 * @param curr 当前帧
 * @param outs 网络前向传播输出结果
 */
    void Segment::postprocess(Frame::Ptr curr, const vector<Mat> &outs)
    {
        Mat outDetections = outs[0];
        Mat outMasks = outs[1];

        // Output size of masks is NxCxHxW where
        // N - number of detected boxes
        // C - number of classes (excluding background)
        // HxW - segmentation shape
        const int numDetections = outDetections.size[2];

        vector<int> classIds;
        vector<float> confidences;
        vector<cv::Rect> boxes;

        int objId = 0;
        outDetections = outDetections.reshape(1, outDetections.total() / 7);
        for (int i = 0; i < numDetections; ++i)
        {
            float score = outDetections.ptr<float>(i)[2];
            int classId = static_cast<int>(outDetections.ptr<float>(i)[1]);
            if (score > Param::segment_thr_ && classId < 9) // 前9类物体视为可能的动态物体
            {
                if (classId == 4 || classId == 6 || classId == 8) // airplane,train,boat剔除
                    continue;
                else if (classId == 3) // motocycle=bicycle
                    classId = 1;
                else if (classId == 5 || classId == 7) // car=bus=truck
                    classId = 2;
                // 提取边界框
                int left = static_cast<int>(Camera::width_ * outDetections.ptr<float>(i)[3]);
                int top = static_cast<int>(Camera::height_ * outDetections.ptr<float>(i)[4]);
                int right = static_cast<int>(Camera::width_ * outDetections.ptr<float>(i)[5]);
                int bot = static_cast<int>(Camera::height_ * outDetections.ptr<float>(i)[6]);

                left = max(0, min(left, Camera::width_ - 1));
                top = max(0, min(top, Camera::height_ - 1));
                right = max(0, min(right, Camera::width_ - 1));
                bot = max(0, min(bot, Camera::height_ - 1));

                classIds.push_back(classId);
                confidences.push_back((float)score);
                boxes.push_back(Rect(left, top, right - left, bot - top));
            }
        }

        // 非极大值抑制
        vector<int> indices;
        NMSBoxes(boxes, confidences, Param::segment_thr_, Param::segment_nmsthr_, indices);

        // 生成目标并加入到当前帧
        for (int i = 0; i < indices.size(); i++)
        {
            int id = indices[i];
            int classId = classIds[id];
            int left = boxes[id].x;
            int right = boxes[id].x + boxes[id].width;
            int top = boxes[id].y;
            int bot = boxes[id].y + boxes[id].height;
            float score = confidences[id];

            // 提取目标掩码
            objId++;
            Mat objectMask(outMasks.size[2], outMasks.size[3], CV_32F, outMasks.ptr<float>(i, classId));
            resize(objectMask, objectMask, Size(right - left + 1, bot - top + 1));
            Mat mask = (objectMask > Param::segment_maskthr_);
            mask.convertTo(mask, CV_8UC1, 1, -255 + objId);
            int area = cv::countNonZero(mask);

            // 生成目标并加入到当前帧中
            Object::Ptr obj(new myslam::Object);
            obj->left_ = left;
            obj->right_ = right;
            obj->top_ = top;
            obj->bot_ = bot;
            obj->score_ = score;
            obj->mask_ = mask;
            obj->area_ = area;
            obj->class_id_ = classId;
            obj->label_ = classes_[classId];
            obj->r_ = rand() % (255 + 1);
            obj->b_ = rand() % (255 + 1);
            obj->g_ = rand() % (255 + 1);
            curr->objects_.push_back(obj);

            curr->mobjs_matched_ = vector<Object::Ptr>(curr->objects_.size(), nullptr);
        }
    }

    /**
 * @brief 制作当前帧目标掩码
 * 
 * @param curr 
 */
    void Segment::maskOBjMask(Frame::Ptr curr)
    {
        for (auto obj : curr->objects_)
        {
            Rect box(obj->left_, obj->top_, obj->right_ - obj->left_ + 1, obj->bot_ - obj->top_ + 1);
            obj->mask_.copyTo(curr->mask_(box), obj->mask_);
        }
    }
} // namespace myslam