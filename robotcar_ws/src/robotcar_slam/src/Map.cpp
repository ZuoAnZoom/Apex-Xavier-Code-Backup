#include "myslam/Map.h"

namespace myslam
{
    Map::Map() : num_mpoints_(0), num_frames_(0), num_objects_(0)
    {
    }

    Map::~Map()
    {
    }

    Frame::Ptr Map::getLastFrame()
    {
        return frames_.back();
    }

    vector<Vec3> Map::getAllMapPointsCoors()
    {
        vector<Vec3> mpoints_coors(map_points_.size());
        int i = 0;
        for (auto it = map_points_.begin(); it != map_points_.end(); it++, i++)
        {
            mpoints_coors[i] = it->get()->coor_;
        }
        return mpoints_coors;
    }

    vector<SE3> Map::getAllFramesPoses()
    {
        vector<SE3> frames_poses(frames_.size());
        int i = 0;
        for (auto it = frames_.begin(); it != frames_.end(); it++, i++)
        {
            frames_poses[i] = it->get()->T_wc_;
        }
        return frames_poses;
    }

    vector<vector<Vec3>> Map::getAllObjectsCoors()
    {
        vector<vector<Vec3>> objects_coors(objects_.size());
        int i = 0;
        for (auto it = objects_.begin(); it != objects_.end(); it++, i++)
        {
            Object::Ptr obj = *it;
            if (!obj->error_)
            {
                objects_coors[i] = obj->corns_wld_coor_;
                objects_coors[i].push_back(Vec3(obj->r_, obj->g_, obj->b_)); // 加入该目标的颜色
            }
        }
        return objects_coors;
    }

    vector<vector<Vec3>> Map::getAllObjectsTrajs()
    {
        vector<vector<Vec3>> objects_trajs(objects_.size());
        int i = 0;
        for (auto it = objects_.begin(); it != objects_.end(); it++, i++)
        {
            if (!it->get()->error_)
                objects_trajs[i] = it->get()->trajectory_;
        }
        return objects_trajs;
    }
} // namespace myslam
