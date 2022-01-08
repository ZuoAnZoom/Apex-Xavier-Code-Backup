#include "myslam/Transform.h"

namespace myslam
{
    // 注意判断计算的相机坐标深度是否小于1，若是则舍弃该点
    Vec3 Transform::world2camera(const Vec3 &p_w, const SE3 &T_w_c)
    {
        return T_w_c.inverse() * p_w;
    }
    Mat Transform::world2camera(const Mat &p_w, const Mat &T_w_c)
    {
        Mat Rt = T_w_c.rowRange(0, 3).t();
        return Rt * p_w - Rt * T_w_c.col(3);
    }

    Vec3 Transform::camera2world(const Vec3 &p_c, const SE3 &T_w_c)
    {
        return T_w_c * p_c;
    }

    Vec2 Transform::camera2pixel(const Mat33 &K, const Vec3 &p_c)
    {
        return Vec2(
            K(0, 0) * p_c(0, 0) / p_c(2, 0) + K(0, 2),
            K(1, 1) * p_c(1, 0) / p_c(2, 0) + K(1, 2));
    }
    Point2f Transform::camera2pixel(const Mat &K, const Mat &p_c)
    {
        return cv::Point2f(K.ptr<float>(0)[0] * p_c.ptr<float>(0)[0] / p_c.ptr<float>(2)[0] + K.ptr<float>(0)[2], K.ptr<float>(1)[1] * p_c.ptr<float>(1)[0] / p_c.ptr<float>(2)[0] + K.ptr<float>(1)[2]);
    }

    Vec3 Transform::pixel2camera(const Mat33 &K, const Vec2 &p_p, const double depth)
    {
        return Vec3(
            (p_p(0, 0) - K(0, 2)) * depth / K(0, 0),
            (p_p(1, 0) - K(1, 2)) * depth / K(1, 1),
            depth);
    }

    Vec3 Transform::pixel2camera(const Mat33 &K, const Point2f &p_p, const double depth)
    {
        return Vec3(
            (p_p.x - K(0, 2)) * depth / K(0, 0),
            (p_p.y - K(1, 2)) * depth / K(1, 1),
            depth);
    }

    Vec3 Transform::pixel2homo(const Mat33 &K, const Vec2 p_p)
    {
        return Vec3(
            (p_p(0, 0) - K(0, 2)) / K(0, 0),
            (p_p(1, 0) - K(1, 2)) / K(1, 1),
            1);
    }

    Vec3 Transform::pixel2homo(const Mat33 &K, const Point2f p_p)
    {
        return Vec3(
            (p_p.x - K(0, 2)) / K(0, 0),
            (p_p.y - K(1, 2)) / K(1, 1),
            1);
    }

    Vec2 Transform::world2pixel(const Mat33 &K, const Vec3 &p_w, const SE3 &T_w_c)
    {
        Vec3 camcoor = world2camera(p_w, T_w_c);
        if (camcoor(2) < 1.0)
            return Vec2(-1, -1);
        return camera2pixel(K, camcoor);
    }
    Point2f Transform::world2pixel(const Mat &K, const Mat &p_w, const Mat &T_w_c)
    {
        Mat camcoor = world2camera(p_w, T_w_c);
        if (camcoor.ptr<float>(2)[0] < 1.0)
            return Point2f(-1, -1);
        return camera2pixel(K, camcoor);
    }

    Vec3 Transform::pixel2world(const Mat33 &K, const Vec2 &p_p, const SE3 &T_w_c, const double depth)
    {
        return camera2world(pixel2camera(K, p_p, depth), T_w_c);
    }

    Vec3 Transform::pixel2world(const Mat33 &K, const Point2f &p_p, const SE3 &T_w_c, const double depth)
    {
        return camera2world(pixel2camera(K, p_p, depth), T_w_c);
    }

    // opencv的旋转和平移矩阵转换为Sophus格式的位姿
    SE3 Transform::cvT2SE3(const Mat &R, const Mat &t)
    {
        cv::Mat cvT;
        cv::hconcat(R, t, cvT);
        Mat34 egT;
        cv::cv2eigen(cvT, egT);
        Eigen::Quaterniond q(egT.block<3, 3>(0, 0));
        q.normalized();
        return SE3(q, egT.col(3));
    }

    /**
    * @brief 计算F矩阵
    * 
    * @param T1 位姿1
    * @param T2 位姿2
    * @param F_21 2到1的基础矩阵F
    */
    void Func::calcFundmental(const SE3 &T1, const SE3 &T2, cv::Mat &F_21)
    {
        SE3 T_21 = T2.inverse() * T1;
        Mat33 E = SO3::hat(T_21.translation()) * T_21.rotationMatrix();
        Mat33 K_i = Camera::K_.inverse();
        Mat33 F = K_i.transpose() * E * K_i;
        cv::eigen2cv(F, F_21);
    }

    void Func::calcReltvRt(const SE3 &T1, const SE3 &T2, float &R_reltv, float &t_reltv)
    {
        // 计算相对位移
        t_reltv = Vec3(T1.translation() - T2.translation()).norm();

        // 计算相对旋转
        SO3 R1 = T1.rotationMatrix();
        SO3 R2 = T2.rotationMatrix();
        R_reltv = (R1.inverse() * R2).log().norm();
    }

    void Func::judgeForOrBackward(const SE3 &T_last, const SE3 &T_curr, bool &isForward, bool &isBackward)
    {
        Vec3 t_reltv = T_curr.translation() - T_last.translation();
        Vec3 t_lc = T_last.so3().inverse() * t_reltv;

        isForward = t_lc(2) > Camera::base_;   // 前进
        isBackward = -t_lc(2) > Camera::base_; // 后退
    }

    float Func::getPixelValue(const cv::Mat &img, const float x0, const float y0)
    {
        float x = x0, y = y0;
        // boundary check
        if (x < 0)
            x = 0;
        if (y < 0)
            y = 0;
        if (x >= img.cols)
            x = img.cols - 1;
        if (y >= img.rows)
            y = img.rows - 1;
        uchar *data = &img.data[int(y) * img.step + int(x)];

        float xx = x - floor(x);
        float yy = y - floor(y);
        return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]);
    }

    // 创建numlayers层金字塔，每层缩放倍数为pyramid_scale < 1，第一层是原图像
    void Func::createPyramids(const int &numlayers, const double pyramid_scale, const Mat &img1, vector<Mat> &pyr)
    {
        for (int i = 0; i < numlayers; i++)
        {
            if (i == 0)
            {
                pyr.push_back(img1);
            }
            else
            {
                cv::Mat img_pyr;
                cv::resize(pyr[i - 1], img_pyr,
                           cv::Size(pyr[i - 1].cols * pyramid_scale, pyr[i - 1].rows * pyramid_scale));
                pyr.push_back(img_pyr);
            }
        }
    }

    // 判断一个点是否在图像边界内
    bool Func::inBorder(const cv::Point2f &pt, int border_size)
    {
        return (Camera::min_X_ + border_size) < pt.x && pt.x < (Camera::max_X_ - border_size) && (Camera::min_Y_ + border_size) < pt.y && pt.y < (Camera::max_Y_ - border_size);
    }
    bool Func::inBorder(const float ptx, const float pty, int border_size)
    {
        return (Camera::min_X_ + border_size) < ptx && ptx < (Camera::max_X_ - border_size) && (Camera::min_Y_ + border_size) < pty && pty < (Camera::max_Y_ - border_size);
    }
    // 判断一个目标边框是否在图像边界内
    bool Func::inBorder(const float left, const float right, const float top, const float bot, int border_size)
    {
        float minX = Camera::min_X_ + border_size;
        float minY = Camera::min_Y_ + border_size;
        float maxX = Camera::max_X_ - border_size;
        float maxY = Camera::max_Y_ - border_size;

        return !(left > maxX || top > maxY || right < minX || bot < minY || (left <= minX && right >= maxX && top <= minY && bot >= maxX));
    }

    // 计算两个像素的距离
    float Func::calcDistence(const cv::Point2f &p0, const cv::Point2f &p1)
    {
        return sqrt((p0.x - p1.x) * (p0.x - p1.x) + (p0.y - p1.y) * (p0.y - p1.y));
    }

    /**
 * @brief 计算两个矩形的交并比IOU
 * 
 * @param R1LeftUp 第一个矩形的左上角坐标
 * @param R1RightDown 第一个矩形的右下角坐标
 * @param R2LeftUp 第二个矩形的左上角坐标
 * @param R2RightDown 第二个矩形的右下角坐标
 * @return float 交并比IOU
 */
    float Func::calcIOU(const cv::Point2f &R1LeftUp, const cv::Point2f &R1RightDown, const cv::Point2f &R2LeftUp, const cv::Point2f &R2RightDown)
    {
        float W = std::min(R2RightDown.x, R1RightDown.x) - std::max(R2LeftUp.x, R1LeftUp.x);
        float H = std::min(R2RightDown.y, R1RightDown.y) - std::max(R2LeftUp.y, R1LeftUp.y);
        if (W <= 0 || H <= 0)
            return 0;
        float SR1 = (R1RightDown.x - R1LeftUp.x) * (R1RightDown.y - R1LeftUp.y);
        float SR2 = (R2RightDown.x - R2LeftUp.x) * (R2RightDown.y - R2LeftUp.y);
        float SU = W * H;
        return (SU) / (SR1 + SR2 - SU);
    }

    //图像块仿射变换
    void Func::getWarpAffine(
        const SE3 &cam_ref, ///关键帧的位姿
        const SE3 &cam_cur, ///当前帧的位姿
        const Mat &patch,
        const Vec2 &px_ref,  ///特征的像素坐标
        const double &depth, ///深度
        Mat &dst_warp        ///仿射变换后的图像
    )
    {
        const int halfpatch_size = 5;

        Vec3 xyz_ref = Transform::pixel2world(Camera::K_, px_ref, cam_ref, depth);
        Vec3 xyz_du_ref = Transform::pixel2world(Camera::K_, px_ref + Vec2(halfpatch_size, 0), cam_ref, depth);
        Vec3 xyz_dv_ref = Transform::pixel2world(Camera::K_, px_ref + Vec2(0, halfpatch_size), cam_ref, depth);

        const Vec2 px_cur(Transform::world2pixel(Camera::K_, xyz_ref, cam_cur));
        const Vec2 px_du(Transform::world2pixel(Camera::K_, xyz_du_ref, cam_cur));
        const Vec2 px_dv(Transform::world2pixel(Camera::K_, xyz_dv_ref, cam_cur));

        Point2f srcPoints[3];
        Point2f dstPoints[3];

        srcPoints[0] = Point2f(px_cur(0, 0), px_cur(0, 1));
        srcPoints[1] = Point2f(px_cur(0, 0) + halfpatch_size, px_cur(0, 1));
        srcPoints[2] = Point2f(px_cur(0, 0), px_cur(0, 1) + halfpatch_size);

        dstPoints[0] = Point2f(px_cur(0, 0), px_cur(0, 1));
        dstPoints[1] = Point2f(px_du(0, 0), px_du(0, 1));
        dstPoints[2] = Point2f(px_dv(0, 0), px_dv(0, 1));

        Mat A_cur_ref = getAffineTransform(srcPoints, dstPoints);

        warpAffine(patch, dst_warp, A_cur_ref, patch.size()); //仿射变换
    }
    /**
 * @brief 三角化一个地图点坐标
 * 
 * @param P_1 第一个相机的投影矩阵
 * @param P_2 第二个相机的投影矩阵
 * @param p_1 第一帧点的像素坐标
 * @param p_2 第二帧点的像素坐标
 * @param p_esti 点的世界坐标
 * // TODO：会有深度为负的点
 */
    void Func::triangulatePoint(const Mat34 &P_1, const Mat34 &P_2, const cv::Point2f &p_1, const cv::Point2f &p_2, Vec3 &p_esti)
    {
        Mat44 H = Mat44::Zero();
        H.row(0) = p_1.y * P_1.row(2) - P_1.row(1);
        H.row(1) = P_1.row(0) - p_1.x * P_1.row(2);
        H.row(2) = p_2.y * P_2.row(2) - P_2.row(1);
        H.row(3) = P_2.row(0) - p_2.x * P_2.row(2);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Mat44 V = svd.matrixV();
        p_esti = V.block(0, 3, 3, 1);

        // 转换成非齐次坐标
        p_esti = p_esti / V(3, 3);
    }
    void Func::triangulatePoint(const Mat &P_1, const Mat &P_2, const cv::Point2f &p_1, const cv::Point2f &p_2, Mat &p_esti)
    {
        Mat H(4, 4, CV_32F, cv::Scalar(0));
        H.row(0) = p_1.y * P_1.row(2) - P_1.row(1);
        H.row(1) = P_1.row(0) - p_1.x * P_1.row(2);
        H.row(2) = p_2.y * P_2.row(2) - P_2.row(1);
        H.row(3) = P_2.row(0) - p_2.x * P_2.row(2);

        Mat u, w, vt;
        cv::SVD::compute(H, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        p_esti = vt.row(3).t();

        // 转换成非齐次坐标
        p_esti = p_esti.rowRange(0, 3) / p_esti.ptr<float>(3)[0];
    }

    /**
 * @brief 计算两个像素块的归一化相似度NCC
 * 
 * @param p1 像素块1
 * @param p2 像素块2
 * @return float NCC
 */
    float Func::calcPatchNcc(const Mat &p1, const Mat &p2)
    {
        float numerator = p1.dot(p2);
        float denominator1 = p1.dot(p1);
        float denominator2 = p2.dot(p2);
        float denominator = sqrt(denominator1 * denominator2);
        return numerator / (denominator + 1e-10); // 防止分母为零
    }
} // namespace myslam