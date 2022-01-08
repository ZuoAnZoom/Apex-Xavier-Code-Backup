#pragma once

#ifndef OPTIMIZATION_MOD
#define OPTIMIZATION_MOD

#include "myslam/CommonInclude.h"
#include "myslam/Transform.h"
#include "myslam/Frame.h"
#include "myslam/MapPoint.h"
#include "myslam/Map.h"

namespace myslam
{
    class Frame;
    class MapPoint;
    class Map;

    class Optimization
    {
    public:
        typedef std::shared_ptr<Optimization> Ptr;
        vector<shared_ptr<Frame>> frames_;        // 要优化的帧
        vector<shared_ptr<MapPoint>> map_points_; // 优化帧观测到的地图点
        vector<shared_ptr<Frame>> fixed_frames_;  // 要固定的帧

        unordered_set<int> fm_id_; // 要优化的帧ID
        unordered_set<int> mp_id_; // 要优化的地图点ID
        unordered_set<int> fx_id_; // 要固定的帧ID

        clock_t t1_, t2_;

    public:
        Optimization();
        ~Optimization();

        void selectVariables(Map::Ptr local_map); // 从局部地图中选出要优化的帧和地图点

        void optimiseMap(Frame::Ptr frame, Map::Ptr local_map); // 优化的主函数

        void optimiseCurrPose(Frame::Ptr pFrame); // 当前帧位姿优化

        // 局部地图优化
        void optimiseLocalMap(vector<Frame::Ptr> lLocalKeyFrames,
                              vector<MapPoint::Ptr> lLocalMapPoints,
                              vector<Frame::Ptr> lFixedCameras);

        void eraseOutliers(
            vector<g2o::EdgeStereoSE3ProjectXYZ *> edge, vector<pair<Frame::Ptr, int>> frame_id,
            vector<pair<MapPoint::Ptr, int>> mpoint_id);
    };

    /// vertex and edges used in g2o ba
    /// 位姿顶点
    class VertexPose : public g2o::BaseVertex<6, SE3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual void setToOriginImpl() override { _estimate = SE3(); }

        /// left multiplication on SE3
        virtual void oplusImpl(const double *update) override
        {
            Vec6 update_eigen;
            update_eigen << update[0], update[1], update[2], update[3], update[4],
                update[5];
            _estimate = SE3::exp(update_eigen) * _estimate;
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
    };

    /// 路标顶点
    class VertexXYZ : public g2o::BaseVertex<3, Vec3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void setToOriginImpl() override { _estimate = Vec3::Zero(); }

        virtual void oplusImpl(const double *update) override
        {
            _estimate[0] += update[0];
            _estimate[1] += update[1];
            _estimate[2] += update[2];
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
    };

    /// 仅估计位姿的一元边
    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K)
            : _pos3d(pos), _K(K) {}

        virtual void computeError() override
        {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            SE3 T = v->estimate();
            Vec3 pos_pixel = _K * (T * _pos3d);
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        virtual void linearizeOplus() override
        {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            SE3 T = v->estimate();
            Vec3 pos_cam = T * _pos3d;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                -fy * X * Zinv;
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }

    private:
        Vec3 _pos3d;
        Mat33 _K;
    };

    /// 位姿图中的两个位姿之间的二元边

    /// 带有地图和位姿的二元边
    class EdgeProjection
        : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /// 构造时传入相机内外参
        EdgeProjection(const Mat33 &K, const SE3 &cam_ext) : _K(K)
        {
            _cam_ext = cam_ext;
        }

        virtual void computeError() override
        {
            const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
            const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
            SE3 T = v0->estimate();
            Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
            pos_pixel /= pos_pixel[2];
            _error = _measurement - pos_pixel.head<2>();
        }

        virtual void linearizeOplus() override
        {
            const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
            const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
            SE3 T = v0->estimate();
            Vec3 pw = v1->estimate();
            Vec3 pos_cam = _cam_ext * T * pw;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            // 重投影误差关于相机位姿李代数的导数
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                -fy * X * Zinv;
            // 重投影误差关于空间点坐标的导数
            _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                               _cam_ext.rotationMatrix() * T.rotationMatrix();
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }

    private:
        Mat33 _K;
        SE3 _cam_ext;
    };

    // project a 3d point into an image plane, the error is photometric error
    // an unary edge with one vertex SE3Expmap (the pose of camera)
    class EdgeSE3ProjectDirect : public BaseUnaryEdge<1, double, VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectDirect() {}

        EdgeSE3ProjectDirect(Vec3 point, float fx, float fy, float cx, float cy, cv::Mat *image)
            : x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image)
        {
        }

        virtual void computeError()
        {
            const VertexSE3Expmap *v = static_cast<const VertexSE3Expmap *>(_vertices[0]);
            Vec3 x_local = v->estimate().map(x_world_);
            float x = x_local[0] * fx_ / x_local[2] + cx_;
            float y = x_local[1] * fy_ / x_local[2] + cy_;
            // check x,y is in the image
            if (x - 4 < 0 || (x + 4) > image_->cols || (y - 4) < 0 || (y + 4) > image_->rows)
            {
                _error(0, 0) = 0.0;
                this->setLevel(1);
            }
            else
            {
                _error(0, 0) = Func::getPixelValue(*image_, x, y) - _measurement;
            }
        }

        // plus in manifold
        virtual void linearizeOplus()
        {
            if (level() == 1)
            {
                _jacobianOplusXi = Mat16::Zero();
                return;
            }
            VertexSE3Expmap *vtx = static_cast<VertexSE3Expmap *>(_vertices[0]);
            Vec3 xyz_trans = vtx->estimate().map(x_world_); // q in book

            double x = xyz_trans[0];
            double y = xyz_trans[1];
            double invz = 1.0 / xyz_trans[2];
            double invz_2 = invz * invz;

            float u = x * fx_ * invz + cx_;
            float v = y * fy_ * invz + cy_;

            // jacobian from se3 to u,v
            // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega is so(3) and \epsilon the translation
            // u/q:投影方程关于相机坐标系下的三维点的导数，q/ksai:变换后的三维点对变换的导数
            Mat26 jacobian_uv_ksai;

            jacobian_uv_ksai(0, 0) = -x * y * invz_2 * fx_;
            jacobian_uv_ksai(0, 1) = (1 + (x * x * invz_2)) * fx_;
            jacobian_uv_ksai(0, 2) = -y * invz * fx_;
            jacobian_uv_ksai(0, 3) = invz * fx_;
            jacobian_uv_ksai(0, 4) = 0;
            jacobian_uv_ksai(0, 5) = -x * invz_2 * fx_;

            jacobian_uv_ksai(1, 0) = -(1 + y * y * invz_2) * fy_;
            jacobian_uv_ksai(1, 1) = x * y * invz_2 * fy_;
            jacobian_uv_ksai(1, 2) = x * invz * fy_;
            jacobian_uv_ksai(1, 3) = 0;
            jacobian_uv_ksai(1, 4) = invz * fy_;
            jacobian_uv_ksai(1, 5) = -y * invz_2 * fy_;

            // I/u:u处的像素梯度
            Mat12 jacobian_pixel_uv;

            jacobian_pixel_uv(0, 0) = (Func::getPixelValue(*image_, u + 1, v) - Func::getPixelValue(*image_, u - 1, v)) / 2;
            jacobian_pixel_uv(0, 1) = (Func::getPixelValue(*image_, u, v + 1) - Func::getPixelValue(*image_, u, v - 1)) / 2;

            _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;
        }

        // dummy read and write functions because we don't care...
        virtual bool read(std::istream &in) {}
        virtual bool write(std::ostream &out) const {}

    public:
        Vec3 x_world_;                            // 3D point in world frame
        float cx_ = 0, cy_ = 0, fx_ = 0, fy_ = 0; // Camera intrinsics
        cv::Mat *image_ = nullptr;                // reference image
    };

} // namespace myslam

#endif