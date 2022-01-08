#include "myslam/Mapping_mod.h"

namespace myslam
{

    Mapping::Mapping()
    {
    }
    /**
 * @brief 更新地图、维护地图和局部地图优化
 * 新关键帧的特征点包含与已经存在的地图点匹配上的匹配地图点 和 未匹配上地图点的新检测到的特征点
 * 
 * 1.建立关键帧与地图点的连接关系：将新关键帧加入到局部地图和全局地图中，将匹配地图点加入到该关键帧的匹配地图数组中，将未匹配特征点恢复出世界坐标，生成地图点后加入到该关键帧的匹配地图数组中
 * 
 * 2.建立地图点与关键帧的连接关系：该关键帧单独生成的地图点加入到局部地图和全局地图中，并将该关键帧加入到匹配地图点的匹配关键帧数组中，并更新匹配地图点的最佳描述子和观测方向
 * 
 * 3.建立关键帧与关键帧的连接关系：取出新关键帧对应的全部地图点，遍历每个地图点，根据该地图点与其他关键帧的共视关系来更新新关键帧与其他关键帧的连接权重
 * 
 * 4.关键帧剔除：根据关键帧之间的共视关系剔除冗余的关键帧，将关键帧的数量维持在一个恒定范围内
 * 
 * 5.地图点剔除：被剔除的关键帧所单独生成的地图点也被剔除，其他与关键帧共视较弱的地图点也被剔除
 * 
 * 6.局部地图优化：对新插入的关键帧完成以上操作后，对局部地图进行一次优化
 * 
 * 7.更新全局地图：将处理完的局部地图更新全局地图
 * 输入:
 * 输出:
 * @param curr 当前帧
 * @param last 上一帧
 * @param map 
*/
    void Mapping::buildMap(Frame::Ptr curr, Frame::Ptr last, Map::Ptr local_map)
    {
        curr_ = curr;

        if (curr_->is_key_)
        {
            // 局部地图中插入当前帧
            insertFrame(local_map);

            // 局部地图中插入当前帧观测到的地图点
            insertMapPoints(local_map);

            // 维护帧和地图点及其共视关系
            if (local_map->frames_.size() > Param::map_size_)
            {
                eraseFrames(local_map);
                eraseMapPoints(local_map);
                updateConnection(local_map);
            }
        }

        // 更新局部地图中的目标属性
        updateObjects(last, local_map);

        cout << "局部地图的规模：   帧数：" << local_map->num_frames_ << "   地图点数：" << local_map->num_mpoints_ << "    目标数：" << local_map->objects_.size() << endl;
    }
    /**
 * @brief 向局部地图中插入当前帧
 * 
 * @param local_map 
 */
    void Mapping::insertFrame(Map::Ptr local_map)
    {
        local_map->frames_.push_back(curr_);
        local_map->num_frames_++;
    }

    /**
 * @brief 向局部地图中插入当前帧所观测到的地图点或更新匹配到的地图点的属性
 * 
 * @param local_map 
 */
    void Mapping::insertMapPoints(Map::Ptr local_map)
    {
        int num_new_mapoints = 0;
        for (int i = 0; i < curr_->features_.size(); i++)
        {
            Feature::Ptr ft = curr_->features_[i];
            MapPoint::Ptr mp = curr_->mpoints_matched_[i];
            // 如果匹配上了地图点，则更新该地图点的属性
            if (mp != nullptr && !mp->dyna_ && !ft->dyna_)
            {
                /*建立关联*/
                mp->obs_kfms_.push_back(curr_);
                mp->obs_kfms_id_.insert(pair<shared_ptr<Frame>, int>(curr_, i));

                /*更新属性*/
                // 三角化该地图点坐标
                trianMapPointCoor(mp, i);

                // 计算平均观测方向
                Vec3 view_agl = (mp->coor_ - curr_->T_wc_.translation()).normalized();
                mp->viewagls_.push_back(view_agl);
                if (mp->viewagls_.size() > Param::map_size_)
                    mp->viewagls_.pop_front();
                mp->computeMeanViewAgl();

                // 计算最佳描述子
                cv::Mat descr = ft->descriptor_;
                mp->descrs_.push_back(descr);
                if (mp->descrs_.size() > Param::map_size_)
                    mp->descrs_.pop_front();
                mp->computeBestDescr();
            }
            // 如果没有匹配上地图点，则生成新地图点
            else if (ft->stereo_ && !ft->dyna_)
            {
                // 生成新地图点
                MapPoint::Ptr new_mp = MapPoint::createMapPoint(curr_, i);

                // 建立关联
                curr_->mpoints_matched_[i] = new_mp;
                new_mp->obs_kfms_.push_back(curr_);
                new_mp->obs_kfms_id_.insert(pair<shared_ptr<Frame>, int>(curr_, i));

                // 插入局部地图
                local_map->map_points_.push_back(new_mp);
                local_map->num_mpoints_++;
                num_new_mapoints++;
            }
        }
        cout << "新生成的局部地图点的数量：" << num_new_mapoints << endl;
    }

    /**
     * @brief 更新局部地图中的目标属性
     * 
     * @param local_map 
     */
    void Mapping::updateObjects(Frame::Ptr last, Map::Ptr local_map)
    {
        // 遍历当前帧目标，更新局部地图的目标
        for (int i = 0; i < curr_->objects_.size(); i++)
        {
            Object::Ptr curr_obj = curr_->objects_[i];
            Object::Ptr map_obj = curr_->mobjs_matched_[i];
            if (curr_obj->error_)
                continue;

            // 新检测到的目标加入到局部地图中
            if (curr_->id_ == 0 || map_obj == nullptr)
            {
                curr_obj->obs_kfms_.push_back(curr_->id_);

                local_map->objects_.push_back(curr_obj);
                local_map->num_objects_++;
            }
            // 与地图目标匹配上的目标更新其属性
            else
            {
                map_obj->obs_kfms_.push_back(curr_->id_); // 观测到的帧

                // 在当前帧的位置
                map_obj->left_ = curr_obj->left_;
                map_obj->top_ = curr_obj->top_;
                map_obj->right_ = curr_obj->right_;
                map_obj->bot_ = curr_obj->bot_;

                map_obj->wld_coor_ = curr_obj->wld_coor_;             // 位置
                map_obj->trajectory_.push_back(curr_obj->wld_coor_);  // 轨迹
                map_obj->corns_wld_coor_ = curr_obj->corns_wld_coor_; // 角点坐标

                map_obj->error_ = curr_obj->error_;
                map_obj->dyna_ = curr_obj->dyna_;
            }
        }
        // 剔除目标
        for (auto iter = local_map->objects_.begin(); iter != local_map->objects_.end();)
        {
            Object::Ptr map_obj = *iter;
            int last_id = map_obj->obs_kfms_.back(); // 最后一次被匹配到的帧ID
            // 统计该目标连续未被匹配的次数
            if (!map_obj->matched_by_curr_)
            {
                // map_obj->trajectory_.push_back(Vec3(0, 0, 0)); // 轨迹
                map_obj->num_unmatched_++;
            }
            else
                map_obj->num_unmatched_ = 0;

            if (map_obj->error_)
            {
                iter = local_map->objects_.erase(iter);
                local_map->num_objects_--;
            }
            // 如果最后一次被观测是10帧之前
            else if (curr_->id_ - last_id > Param::map_size_)
            {
                iter = local_map->objects_.erase(iter);
                local_map->num_objects_--;
            }
            // 如果连续未被匹配的次数超过5次
            else if (map_obj->num_unmatched_ > 5) //// 阈值
            {
                iter = local_map->objects_.erase(iter);
                local_map->num_objects_--;
            }
            else
            {
                iter++;
            }
        }
    }

    /**
 * @brief 剔除冗余关键帧
 * 在小范围场景中不能总是保留最近的一小段关键帧，但是离开原先场景时又不能总保留原先的关键帧
 * 输入:
 * 输出:
 */
    void Mapping::eraseFrames(Map::Ptr local_map)
    {
        del_fm_id_.clear();

        // 判断哪些帧需要删除
        // del_fm_id_.insert(local_map->frames_.front()->id_); // 暂时选局部地图第一帧作为待删除关键帧

        // 删除距离当前帧距离最远的关键帧
        float max_id = -1, max_dist = 0;
        for (auto it = local_map->frames_.begin(); it != local_map->frames_.end(); it++)
        {
            float dist = (it->get()->T_wc_.translation() - curr_->T_wc_.translation()).norm();
            if (dist > max_dist)
            {
                max_id = it->get()->id_;
                max_dist = dist;
            }
        }
        del_fm_id_.insert(max_id);

        // 删除与当前关键帧共视关系最弱的帧
        // 问题：静止的时候全保留下最近的帧、总是会删除最老的帧

        // 如果一个关键帧中超过90%的点能被其他关键帧观测到，则删除该关键帧
        // for (auto it = local_map->frames_.begin(); it != local_map->frames_.end(); it++)
        // {
        //     Frame::Ptr fm = *it;
        //     int num_mpts = 0, valid_mpts = 0;
        //     for (int j = 0; j < fm->mpoints_matched_.size(); j++)
        //     {
        //         MapPoint::Ptr mp = fm->mpoints_matched_[j];
        //         if (mp == nullptr)
        //             continue;
        //         num_mpts++;
        //         if (mp->obs_kfms_.size() > 3)
        //             valid_mpts++;
        //     }
        //     cout << valid_mpts << "  " << 0.7 * num_mpts << endl;
        //     if (valid_mpts > 0.7 * num_mpts)
        //     {
        //         del_fm_id_.insert(fm->id_);
        //     }
        // }

        // 删除局部地图点中共视关系最弱的关键帧
        // 问题：静止的时候全保留下最近的帧、总是会删除最老的帧
        // map<int, int> num_fm_obs;
        // for (auto it = local_map->map_points_.begin(); it != local_map->map_points_.end(); it++)
        // {
        //     MapPoint::Ptr mp = *it;
        //     for (auto it2 = mp->obs_kfms_.begin(); it2 != mp->obs_kfms_.end(); it2++)
        //     {
        //         if (!it2->expired())
        //         {
        //             int id = it2->lock()->id_;
        //             num_fm_obs[id]++;
        //         }
        //     }
        // }
        // int min_id = -1, min_val = INT_MAX;
        // for (auto it = num_fm_obs.begin(); it != num_fm_obs.end(); it++)
        // {
        //     // cout << it->first << "  " << it->second << endl;
        //     if (it->second < min_val)
        //     {
        //         min_val = it->second;
        //         min_id = it->first;
        //     }
        // }
        // // cout << min_id << endl;
        // del_fm_id_.insert(min_id);

        // 删除关键帧
        // local_map->frames_.pop_front();
        int count = 0;
        for (auto it = local_map->frames_.begin(); it != local_map->frames_.end();)
        {
            if (del_fm_id_.find(it->get()->id_) != del_fm_id_.end())
            {
                it = local_map->frames_.erase(it);
                local_map->num_frames_--;
                count++;
                if (count == del_fm_id_.size())
                    break;
            }
            else
                it++;
        }
    }

    /**
     * @brief 剔除冗余地图点
     * 
     * @param local_map 
     */
    void Mapping::eraseMapPoints(Map::Ptr local_map)
    {
        del_mp_id_.clear();

        for (auto it = local_map->map_points_.begin(); it != local_map->map_points_.end();)
        {
            MapPoint::Ptr mp = *it;

            // 删除已经没有共视关键帧的地图点
            int num_obs = mp->obs_kfms_.size();
            if (num_obs == 0)
            {
                it = local_map->map_points_.erase(it);
                del_mp_id_.insert(mp->id_);
                local_map->num_mpoints_--;
                continue;
            }
            // 剔除动态地图点
            else if (mp->dyna_)
            {
                it = local_map->map_points_.erase(it);
                del_mp_id_.insert(mp->id_);
                local_map->num_mpoints_--;
                continue;
            }
            // 被较少匹配到的地图点也被剔除
            else if ((float)mp->matched_times_ / (float)mp->visible_times_ < 0.25)
            {
                it = local_map->map_points_.erase(it);
                del_mp_id_.insert(mp->id_);
                local_map->num_mpoints_--;
                continue;
            }

            // 取第一次和最后一次被观测到的关键帧
            Frame::Ptr fm_front, fm_back;
            if (!mp->obs_kfms_.front().expired() && !mp->obs_kfms_.back().expired())
            {
                fm_front = mp->obs_kfms_.front().lock();
                fm_back = mp->obs_kfms_.back().lock();
            }
            else
            {
                it++;
                continue;
            }

            // 删除要被剔除的关键帧所单独生成的地图点
            if (num_obs == 1 && del_fm_id_.find(fm_front->id_) != del_fm_id_.end())
            {
                it = local_map->map_points_.erase(it);
                del_mp_id_.insert(mp->id_);
                local_map->num_mpoints_--;
                continue;
            }
            // 最后一次被观测到距当前帧过去了10帧则被剔除
            // else if (curr_->id_ - fm_back->id_ > Param::map_size_)
            // {
            //     it = local_map->map_points_.erase(it);
            //     del_mp_id_.insert(mp->id_);
            //     local_map->num_mpoints_--;
            //     continue;
            // }
            else
                it++;
        }
    }

    /**
     * @brief 更新关键帧与地图点之间的共视关系
     * 
     * @param local_map 
     */
    void Mapping::updateConnection(Map::Ptr local_map)
    {
        // 删除地图点与被剔除关键帧之间的共视关系
        for (auto it1 = local_map->map_points_.begin(); it1 != local_map->map_points_.end(); it1++)
        {
            MapPoint::Ptr mp = *it1;
            int count = 0;
            for (auto it2 = mp->obs_kfms_.begin(); it2 != mp->obs_kfms_.end();)
            {
                if (!it2->expired() && del_fm_id_.find(it2->lock()->id_) == del_fm_id_.end())
                {
                    it2++;
                }
                else
                {
                    mp->obs_kfms_id_.erase(*it2);
                    it2 = mp->obs_kfms_.erase(it2);
                    mp->matched_times_--;
                    mp->visible_times_--;
                    count++;
                    if (count == del_fm_id_.size())
                        break;
                }
            }
        }

        // 删除关键帧与被剔除地图点之间的共视关系
        for (auto it1 = local_map->frames_.begin(); it1 != local_map->frames_.end(); it1++)
        {
            Frame::Ptr fm = *it1;
            int count = 0;
            for (int i = 0; i < fm->mpoints_matched_.size(); i++)
            {
                if (fm->mpoints_matched_[i] != nullptr && del_mp_id_.find(fm->mpoints_matched_[i]->id_) != del_mp_id_.end())
                {
                    fm->mpoints_matched_[i] = nullptr;
                    fm->num_mpoints_--;
                    count++;
                    if (count == del_mp_id_.size()) // 为了提前结束遍历
                        break;
                }
            }
        }
    }

    /**
 * @brief 找到能观测到该地图点的最小视角余弦的两帧，并三角化该点
 * 
 * @param mp 地图点
 * @param cur_idx 在当前帧中的索引
 */
    void Mapping::trianMapPointCoor(MapPoint::Ptr mp, int cur_idx)
    {
        /* 两两帧计算视角余弦*/
        weak_ptr<Frame> ref_fm;
        double min_fm_view_cos = 1;
        Vec3 curr_v = mp->coor_ - curr_->T_wc_.translation();
        for (auto iter : mp->obs_kfms_)
        {
            Frame::Ptr fm;
            if (!iter.expired())
                fm = iter.lock();
            else
                continue;

            if (fm == curr_)
                continue;
            // 两帧视角余弦
            Vec3 ref_v = mp->coor_ - fm->T_wc_.translation();
            double fm_view_cos = curr_v.dot(ref_v) / (curr_v.norm() * ref_v.norm());
            // 取最小余弦值
            if (fm_view_cos < min_fm_view_cos)
            {
                min_fm_view_cos = fm_view_cos;
                ref_fm = fm;
            }
        }

        /* 判断是否小于该点的最小观测余弦*/
        if (min_fm_view_cos < mp->min_viewcos_)
        {
            mp->min_viewcos_ = min_fm_view_cos;
        }
        else
            return;

        /* 三角化*/
        // 取出该点在参考帧中的索引
        int ref_idx;
        Frame::Ptr ref;
        auto iter = mp->obs_kfms_id_.find(ref_fm);
        if (iter != mp->obs_kfms_id_.end())
        {
            ref_idx = iter->second;
            if (!iter->first.expired())
                ref = iter->first.lock();
            else
                return;
        }
        else
            return;

        // 计算投影矩阵并三角化
        Mat34 P_1 = Camera::K_ * ref->T_wc_.inverse().matrix3x4();
        Mat34 P_2 = Camera::K_ * curr_->T_wc_.inverse().matrix3x4();
        Vec3 wld_coor;
        Func::triangulatePoint(P_1, P_2, ref->features_[ref_idx]->pt_, curr_->features_[cur_idx]->pt_, wld_coor);
        if (!isfinite(wld_coor(0)) || !isfinite(wld_coor(1)) || !isfinite(wld_coor(2)))
            return;
        // cout << mp->coor_.transpose() << "  " << wld_coor.transpose() << "  " << (mp->coor_ - wld_coor).norm() << endl;

        /* 检验重投影误差是否在合理范围内*/
        float d, u_r, ori_x, ori_y, err_x, err_y, err_x_r, sigma;
        Vec3 camcoor;
        Vec2 pixcoor;
        // 当前帧
        camcoor = Transform::world2camera(wld_coor, curr_->T_wc_);
        d = camcoor(2);
        if (d < 1.0)
            return;
        pixcoor = Transform::camera2pixel(Camera::K_, camcoor);
        u_r = pixcoor(0) - Camera::base_fx_ / d;
        ori_x = curr_->features_[cur_idx]->x_;
        ori_y = curr_->features_[cur_idx]->y_;
        err_x = ori_x - pixcoor(0);
        err_y = ori_y - pixcoor(1);
        err_x_r = curr_->features_[cur_idx]->x_r_ - u_r;
        sigma = curr_->features_[cur_idx]->octave_;
        if ((err_x * err_x + err_y * err_y + err_x_r * err_x_r) >= 7.8 * sigma)
            return;
        // 参考帧
        camcoor = Transform::world2camera(wld_coor, ref->T_wc_);
        d = camcoor(2);
        if (d < 1.0)
            return;
        pixcoor = Transform::camera2pixel(Camera::K_, camcoor);
        u_r = pixcoor(0) - Camera::base_fx_ / d;
        ori_x = ref->features_[ref_idx]->x_;
        ori_y = ref->features_[ref_idx]->y_;
        err_x = ori_x - pixcoor(0);
        err_y = ori_y - pixcoor(1);
        err_x_r = ref->features_[ref_idx]->x_r_ - u_r;
        sigma = ref->features_[ref_idx]->octave_;
        if ((err_x * err_x + err_y * err_y + err_x_r * err_x_r) >= 7.8 * sigma)
            return;

        /* 通过检验*/
        mp->coor_ = wld_coor;
    }

    /**
 * @brief 将处理完的局部地图更新全局地图
 * 
 * @param local_map 
 * @param global_map 
*/
    void Mapping::updateGlobalMap(Map::Ptr local_map, Map::Ptr global_map)
    {
    }
} // namespace myslam
