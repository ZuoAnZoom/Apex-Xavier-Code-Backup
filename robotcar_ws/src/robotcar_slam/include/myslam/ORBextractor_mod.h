#ifndef ORBEXTRACTOR_MOD_H
#define ORBEXTRACTOR_MOD_H

#include "myslam/CommonInclude.h"
#include "myslam/Camera.h"
#include "myslam/Param.h"

namespace myslam
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

    class ExtractorNode
    {
    public:
        ExtractorNode() : bNoMore(false) {}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    class ORBextractor
    {
    public:
        ORBextractor();
        ~ORBextractor();
        void extractORB(cv::Mat _image, vector<KeyPoint> &_keypoints, cv::Mat &_descriptors); // 特征提取
        // void extractORB(cv::Mat _image, vector<KeyPoint> &_keypoints, cv::Mat &_descriptors); // 特征提取
        void calcDescriptors(const vector<KeyPoint> &_keypoints, cv::Mat &_descriptors); // 计算描述子

    public:
        typedef std::shared_ptr<ORBextractor> Ptr;
        // Keypoints are assigned to cells单元格 in a grid to reduce matching complexity when projecting MapPoints.
        // 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;
        // 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀

        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
        static int TH_LOW, TH_HIGH;
        static int HISTO_LENGTH;

        std::vector<cv::Mat> mvImagePyramid;

    public:
        static void AssignfeatoGrid(vector<cv::KeyPoint> &keypoints_l_, ORBextractor::Ptr ORBextractorLeft);

        static bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        vector<size_t> GetFeaturesInArea(const vector<cv::KeyPoint> &keypoints_l, const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1);


        void ComputePyramid(cv::Mat image);

        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);

        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX, const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

        static void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b); //计算两个描述子的Hanming距离
        std::vector<cv::Point> pattern;                                    //用于存放训练的模板

        static int nfeatures; // 每帧提取特征点数量

        static std::vector<int> umax;

        static std::vector<int> mnFeaturesPerLevel;
        static std::vector<float> mvScaleFactor;
        static std::vector<float> mvInvScaleFactor;
        static std::vector<float> mvLevelSigma2;
        static std::vector<float> mvInvLevelSigma2;
        static void initStaticParam();
    };

    // 左目特征点与右目特征点进行匹配
    void matchLeftRight(const vector<cv::KeyPoint> &keypoints_l_, const vector<cv::KeyPoint> &keypoints_r_, vector<float> &left_to_right_,
                        const Mat &descriptors_l_, const Mat &descriptors_r_, ORBextractor::Ptr ORBextractorLeft, ORBextractor::Ptr ORBextractorRight);

} // namespace myslam
#endif //ORBEXTRATOR_H