#include "myslam/MapPoint.h"

namespace myslam
{
    bool Sort::operator()(const weak_ptr<Frame> _A, const weak_ptr<Frame> _B) const
    {
        if (!_A.expired() && !_B.expired())
        {
            if (_A.lock()->id_ < _B.lock()->id_)
                return true;
            else
                return false;
        }
        else
            return false;
    }

    int MapPoint::factory_id_ = 0;

    MapPoint::MapPoint(int id, const Vec3 &coor, const Vec3 &best_viewagl, const Mat &best_descr, double max_d, double min_d, double min_viewcos)
        : id_(id), coor_(coor), mean_viewagl_(best_viewagl), best_descr_(best_descr), max_dist_(max_d), min_dist_(min_d), min_viewcos_(min_viewcos), visible_times_(1), matched_times_(1)
    {
        descrs_.push_back(best_descr_);
        viewagls_.push_back(mean_viewagl_);
    }

    MapPoint::Ptr MapPoint::createMapPoint(shared_ptr<Frame> fm, int i)
    {
        Vec3 coor = fm->features_[i]->wldcoor_;
        Vec3 best_viewagl = (coor - fm->T_wc_.translation()).normalized();
        Feature::Ptr ft = fm->features_[i];
        Mat best_descr = ft->descriptor_;
        int layer = ft->octave_;
        float depth = fm->features_[i]->camcoor_(2, 0);
        float max_dist = depth * pow(Param::scale_, layer);
        float min_dist = depth / (pow(Param::scale_, Param::nlevels_ - 1 - layer));
        float view_cos;
        if (Camera::Type_ == Camera::eType::STEREO || Camera::Type_ == Camera::eType::RGBD)
        {
            Vec3 cam_coor = fm->features_[i]->camcoor_;
            Vec3 left_v = cam_coor;
            Vec3 right_v = cam_coor - Vec3(Camera::base_, 0, 0);
            view_cos = left_v.dot(right_v) / (left_v.norm() * right_v.norm());
        }
        else
        {
            view_cos = fm->features_[i]->view_cos_;
        }
        return MapPoint::Ptr(
            new MapPoint(factory_id_++, coor, best_viewagl, best_descr, max_dist, min_dist, view_cos));
    }
    /**
 * @brief 计算最佳描述子
 * 
*/
    void MapPoint::computeBestDescr()
    {
        const size_t N = descrs_.size();
        vector<vector<float>> distances;
        distances.resize(N, vector<float>(N, 0));
        for (size_t i = 0; i < N; i++)
            for (size_t j = i + 1; j < N; j++)
            {
                int distij = ORBextractor::DescriptorDistance(descrs_[i], descrs_[j]);
                distances[i][j] = distij;
                distances[j][i] = distij;
            }
        int bestmedian = INT_MAX;
        int bestid = 0;
        for (size_t i = 0; i < N; i++)
        {
            vector<int> dists(distances[i].begin(), distances[i].end());
            sort(dists.begin(), dists.end());
            int median = dists[0.5 * (N - 1)];
            if (median < bestmedian)
            {
                bestmedian = median;
                bestid = i;
            }
        }
        best_descr_ = descrs_[bestid].clone();
    }
    /**
 * @brief 计算平均观测方向
 * 
*/
    void MapPoint::computeMeanViewAgl()
    {
        Vec3 normal = Vec3(0, 0, 0);
        for (int i = 0; i < viewagls_.size(); i++)
            normal += viewagls_[i];
        normal = normal.normalized();
        mean_viewagl_ = normal;
    }
} // namespace myslam
