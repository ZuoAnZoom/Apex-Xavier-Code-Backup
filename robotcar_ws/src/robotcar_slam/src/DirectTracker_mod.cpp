#include "myslam/DirectTracker_mod.h"

namespace myslam
{
    vector<float> DirectTracker::scales_;
    
    DirectTracker::DirectTracker()
    {
    }

    DirectTracker::~DirectTracker()
    {
    }

    void DirectTracker::trackSingleLayer(const VecVector2d &px_ref,
                                         SE3 &T21,
                                         VecVector2d &projection,
                                         DirectSolver &solver)
    {
        const int iterations = 10;
        double cost = 0, lastCost = 0;

        for (int iter = 0; iter < iterations; iter++)
        {
            solver.reset();
            cv::parallel_for_(cv::Range(0, px_ref.size()),
                              std::bind(&DirectSolver::accumulate_jacobian, &solver, std::placeholders::_1));
            Mat66 H = solver.hessian();
            Vec6 b = solver.bias();

            // solve update and put it into estimation
            Vec6 update = H.ldlt().solve(b);
            T21 = SE3::exp(update) * T21;
            cost = solver.cost_func();

            if (std::isnan(update[0]))
            {
                // sometimes occurred when we have a black or white patch and H is irreversible
                break;
            }
            if (iter > 0 && cost > lastCost)
            {
                break;
            }
            if (update.norm() < 1e-3)
            {
                // converge
                break;
            }

            lastCost = cost;
        }
        projection = solver.projected_points();
    }
    /**
 * @brief 前后帧直接法
 * 
 * @param camK 
 * @param img1 
 * @param img2 
 * @param px_ref 
 * @param depth_ref 
 * @param T21 
 * @param keypoints_tracked 
 */
    void DirectTracker::trackMultiLayer(const float camK[4],
                                        const cv::Mat &img1,
                                        const cv::Mat &img2,
                                        const VecVector2d &px_ref,
                                        const vector<double> &depth_ref,
                                        SE3 &T21,
                                        vector<cv::Point2f> &keypoints_tracked)
    {
        // 创建金字塔
        vector<cv::Mat> pyr1, pyr2; // image pyramids
        Func::createPyramids(Param::pyramids_, Param::pyramid_scale_, img1, pyr1);
        Func::createPyramids(Param::pyramids_, Param::pyramid_scale_, img2, pyr2);

        // 从金字塔顶层到底层进行跟踪
        float fx, fy, cx, cy;
        VecVector2d projection;
        for (int level = Param::pyramids_ - 1; level >= 0; level--)
        {
            VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
            for (auto &px : px_ref)
            {
                px_ref_pyr.push_back(scales_[level] * px);
            }

            // scale fx, fy, cx, cy in different pyramid levels
            fx = camK[0] * scales_[level];
            fy = camK[1] * scales_[level];
            cx = camK[2] * scales_[level];
            cy = camK[3] * scales_[level];
            float K[4] = {fx, fy, cx, cy};
            DirectSolver solver(Param::half_patch_, K, pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
            trackSingleLayer(px_ref_pyr, T21, projection, solver);
        }

        // 对跟踪结果赋值
        keypoints_tracked.reserve(px_ref.size());
        for (size_t i = 0; i < px_ref.size(); ++i)
        {
            auto p_cur = projection[i];

            cv::Point2f tracked_pos(p_cur[0], p_cur[1]);
            if (Func::inBorder(tracked_pos))
                keypoints_tracked.push_back(tracked_pos);
            else
            {
                keypoints_tracked.push_back(Point2f(0, 0));
            }
        }
    }

    void DirectSolver::accumulate_jacobian(const cv::Range &range)
    {
        int cnt_good = 0;
        Mat66 hessian = Mat66::Zero();
        Vec6 bias = Vec6::Zero();
        double cost_tmp = 0;

        for (size_t i = range.start; i < range.end; i++)
        {
            // compute the projection in the second image
            Vec3 point_ref =
                depth_ref[i] * Vec3((px_ref[i][0] - cx) / fx, (px_ref[i][1] - cy) / fy, 1);
            Vec3 point_cur = T21 * point_ref;
            if (point_cur[2] < 0) // depth invalid
                continue;

            float u = fx * point_cur[0] / point_cur[2] + cx, v = fy * point_cur[1] / point_cur[2] + cy;
            if (u < half_patch_size || u > curr.cols - half_patch_size || v < half_patch_size || v > curr.rows - half_patch_size)
                continue;

            projection[i] = Vec2(u, v);
            double X = point_cur[0], Y = point_cur[1], Z = point_cur[2],
                   Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;
            cnt_good++;

            // and compute error and jacobian
            // for (int x = -half_patch_size; x <= half_patch_size; x++)
            // for (int y = -half_patch_size; y <= half_patch_size; y++) // 按列遍历
            for (int j = 0; j < 9; j++)
            {
                int x = pattern[j][0];
                int y = pattern[j][1];
                double error;

                error = Func::getPixelValue(last, px_ref[i][0] + x, px_ref[i][1] + y) - Func::getPixelValue(curr, u + x, v + y);

                Mat26 J_pixel_xi;
                Vec2 J_img_pixel;

                J_pixel_xi(0, 0) = fx * Z_inv;
                J_pixel_xi(0, 1) = 0;
                J_pixel_xi(0, 2) = -fx * X * Z2_inv;
                J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
                J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
                J_pixel_xi(0, 5) = -fx * Y * Z_inv;

                J_pixel_xi(1, 0) = 0;
                J_pixel_xi(1, 1) = fy * Z_inv;
                J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
                J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
                J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
                J_pixel_xi(1, 5) = fy * X * Z_inv;

                J_img_pixel = Vec2(
                    0.5 * (Func::getPixelValue(curr, u + 1 + x, v + y) - Func::getPixelValue(curr, u - 1 + x, v + y)),
                    0.5 * (Func::getPixelValue(curr, u + x, v + 1 + y) - Func::getPixelValue(curr, u + x, v - 1 + y)));

                // total jacobian
                Vec6 J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();

                hessian += J * J.transpose();
                bias += -error * J;
                cost_tmp += error * error;
            }
        }
        if (cnt_good)
        {
            // set hessian, bias and cost
            unique_lock<mutex> lck(hessian_mutex);
            H += hessian;
            b += bias;
            cost += cost_tmp / cnt_good;
        }
    }
} // namespace myslam