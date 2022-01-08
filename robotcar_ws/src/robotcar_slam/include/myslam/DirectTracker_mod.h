#pragma once

#ifndef DIRECTTRACKER_MOD_H
#define DIRECTTRACKER_MOD_H

#include "myslam/CommonInclude.h"
#include "myslam/Param.h"
#include "myslam/Transform.h"

namespace myslam
{
    class DirectSolver;

    class DirectTracker
    {
    public:
        typedef shared_ptr<DirectTracker> Ptr;

        static vector<float> scales_;

    public:
        DirectTracker();
        ~DirectTracker();

        void trackSingleLayer(const VecVector2d &px_ref,
                              SE3 &T21,
                              VecVector2d &projection,
                              DirectSolver &solver);

        void trackMultiLayer(const float camK[4],
                             const cv::Mat &img1,
                             const cv::Mat &img2,
                             const VecVector2d &px_ref,
                             const vector<double> &depth_ref,
                             SE3 &T21,
                             vector<cv::Point2f> &keypoints_tracked);
    };

    class DirectSolver
    {
    public:
        DirectSolver(
            int half_size_,
            const float camK[4],
            const cv::Mat &last_,
            const cv::Mat &curr_,
            const VecVector2d &px_ref_,
            const vector<double> &depth_ref_,
            SE3 &T21_) : half_patch_size(half_size_), fx(camK[0]), fy(camK[1]), cx(camK[2]), cy(camK[3]), last(last_), curr(curr_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_)
        {
            projection = VecVector2d(px_ref.size(), Vec2(0, 0));
        }

        /// accumulate jacobians in a range
        void accumulate_jacobian(const cv::Range &range);

        /// get hessian matrix
        Mat66 hessian() const { return H; }

        /// get bias
        Vec6 bias() const { return b; }

        /// get total cost
        double cost_func() const { return cost; }

        /// get projected points
        VecVector2d projected_points() const { return projection; }

        /// reset h, b, cost to zero
        void reset()
        {
            H = Mat66::Zero();
            b = Vec6::Zero();
            cost = 0;
        }

    private:
        int pattern[9][2] = {{0, -2}, {-1, -1}, {1, -1}, {-2, 0}, {0, 0}, {2, 0}, {-1, 1}, {1, 1}, {0, 2}};
        int half_patch_size;
        double fx, fy, cx, cy;
        const cv::Mat last;
        const cv::Mat curr;
        const VecVector2d &px_ref;
        const vector<double> depth_ref;
        SE3 &T21;
        VecVector2d projection; // projected points

        std::mutex hessian_mutex;
        Mat66 H = Mat66::Zero();
        Vec6 b = Vec6::Zero();
        double cost = 0;
    };

} // namespace myslam

#endif