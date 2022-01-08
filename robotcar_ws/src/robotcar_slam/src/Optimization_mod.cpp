//
// Created by zhao on 2019/10/4.
//

#include "myslam/Optimization_mod.h"

namespace myslam
{

    Optimization::Optimization()
    {
    }
    Optimization::~Optimization()
    {
    }
    /**
 * @brief 优化当前帧位姿,局部地图和全局地图
 * 
 */
    void Optimization::optimiseMap(Frame::Ptr frame, Map::Ptr local_map)
    {
        if (!frame->is_key_ || local_map->num_frames_ < 10)
            return;

        // 筛选要优化的变量
        selectVariables(local_map);

        // 局部地图优化
        optimiseLocalMap(frames_, map_points_, fixed_frames_);

        frames_.clear();
        map_points_.clear();
        fixed_frames_.clear();
    }

    /**
 * @brief 当前帧位姿优化
 * 
 * @param pFrame 当前帧
 */
    void Optimization::optimiseCurrPose(Frame::Ptr pFrame)
    {
        int nInitialCorrespondences = pFrame->num_mpoints_; //当前帧匹配到的地图点数量
        if (nInitialCorrespondences < 3)
        {
            cout << "当前帧匹配到的地图点数量太少，无法进行优化" << endl;
            return;
        }
        // 步骤1：构造g2o优化器
        //1.线性方程求解器
        //2.矩阵快求解器
        //3.梯度下降方法
        //6：int _PoseDim, 3：int _LandmarkDim   线性求解器
        SparseOptimizer optimizer; // 优化器

        // std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>()); //线性方程求解器LinearSolverDense，使用cholesky分解

        // std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>()); //线性方程求解器LinearSolverCSparse，使用CSparse分解

        std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>()); //线性方程求解器LinearSolverEigen，使用eigen中sparse Cholesky分解

        std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver))); //矩阵块求解器，求解雅克比矩阵和海森矩阵

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr)); //梯度下降方法，迭代算法

        optimizer.setAlgorithm(solver); //设置优化器

        // 步骤2：添加顶点：待优化当前帧的Tcw
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap(); //顶点的类型，也就是继承自BaseVertex等基础类的类！不是顶点的数据类！例如必须是VertexSE3Expmap而不是VertexSE3Expmap的数据类型类SE3Quat。原因的话源码里面也很清楚，因为后面会用到一系列顶点的维度等等的属性，这些属性是数据类型类里面没有的。

        vSE3->setEstimate(g2o::SE3Quat((pFrame->T_wc_.inverse()).rotationMatrix(), (pFrame->T_wc_.inverse()).translation())); //设置待优化位姿（这是粗略位姿）

        vSE3->setId(0); //设置ID,只有一个顶点

        //设置是否固定，是要优化的变量，不是固定值
        vSE3->setFixed(false); //剔除掉固定点（fixed）之后，把所有的顶点按照不被margin 在前，被margin在后的顺序排成vector类型，变量为_ivMap，这个变量很重要，基本上后面的所有程序都是用这个变量进行遍历的

        optimizer.addVertex(vSE3); // 优化器添加顶点

        // 步骤3：添加一元边：相机投影模型
        const int N = pFrame->features_.size(); //N：当前帧左目特征点数量
        vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesStereo.reserve(N);
        vnIndexEdgeStereo.reserve(N);
        const float deltaStereo = sqrt(7.815);
        std::vector<bool> mvbOutlier(N, false); // 该点是否是外点的标志
        {
            for (int i = 0; i < N; i++)
            {
                if (pFrame->mpoints_matched_[i] == nullptr)
                    continue;
                Vec3 pMP = pFrame->mpoints_matched_[i]->coor_; // 匹配到的地图点坐标

                mvbOutlier[i] = false;
                // nInitialCorrespondences++;
                //SET EDGE
                Vec3 obs;
                // const cv::KeyPoint &kpUn = pFrame->keypoints_l_[i];
                const Feature::Ptr ftUn = pFrame->features_[i];
                const float &kp_ur = ftUn->x_r_;
                obs << ftUn->x_, ftUn->y_, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose(); // 边的类型

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0))); // 对边添加顶点

                e->setMeasurement(obs); // 对边添加观测

                const float invSigma2 = ORBextractor::mvInvLevelSigma2[ftUn->octave_];
                Mat33 Info = Mat33::Identity() * invSigma2;
                e->setInformation(Info); // 设置边的信息矩阵，协方差

                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk); // 设置边的鲁棒核函数
                rk->setDelta(deltaStereo);

                e->fx = Camera::fx_; // 设置边的相机内参
                e->fy = Camera::fy_;
                e->cx = Camera::cx_;
                e->cy = Camera::cy_;
                e->bf = Camera::base_fx_;
                e->Xw[0] = pMP(0, 0);
                e->Xw[1] = pMP(1, 0);
                e->Xw[2] = pMP(2, 0);

                optimizer.addEdge(e); // 优化器添加边
                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

        // 步骤4：开始优化，总共优化四次，每次优化后，将观测分为outlier和inlier，outlier不参与下次优化，由于每次优化后是对所有的观测进行outlier和inlier判别，因此之前被判别为outlier有可能变成inlier，反之亦然
        // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        const int its[4] = {10, 10, 10, 10}; // 四次迭代，每次迭代的次数

        int nBad = 0; // 外点数

        for (size_t it = 0; it < 4; it++)
        {
            vSE3->setEstimate(g2o::SE3Quat((pFrame->T_wc_.inverse()).rotationMatrix(), (pFrame->T_wc_.inverse()).translation())); // 设置顶点的估计值

            optimizer.initializeOptimization(0); // 对level为0的边进行优化
            optimizer.optimize(its[it]);         //指定迭代次数

            nBad = 0;
            // mvbOutlier(N, true);
            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (mvbOutlier[idx])
                {
                    e->computeError(); // NOTE g2o只会计算active edge的误差
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    e->setLevel(0);
                    mvbOutlier[idx] = false;
                }

                if (it == 2)
                    e->setRobustKernel(0); // 因为剔除了错误的边，所以下次优化不再使用核函数
            }

            if (optimizer.edges().size() < 10)
                break;
        }

        // 优化后位姿赋值及输出内点数量
        g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        pFrame->T_wc_ = SE3(SE3quat_recov.rotation(), SE3quat_recov.translation()).inverse();
        // pFrame->T_cw_ = pFrame->T_cw_.inverse();
        // cout << "nInitialCorrespondences-nBad= " << nInitialCorrespondences - nBad << endl;
    }

    /**
 * @brief 从局部地图中选出需要优化和固定的变量,包括帧和地图点
 * 
 * @param frame 当前帧
 * @param local_map 局部地图
 */
    void Optimization::selectVariables(Map::Ptr local_map)
    {
        // 选择要优化的帧
        // TODO：设计更复杂的策略来筛选要优化的关键帧
        int count = 0;
        for (auto it = local_map->frames_.begin(); it != local_map->frames_.end(); it++, count++)
        {
            if ((count == 0 || count == 4 || count == 9) && it->get()->id_ != 0)
            {
                frames_.push_back(*it);
                fm_id_.insert(it->get()->id_);
            }
        }

        // 选择要优化的地图点
        for (int i = 0; i < frames_.size(); i++)
        {
            for (auto mp : frames_[i]->mpoints_matched_)
            {
                if (mp == nullptr)
                    continue;
                if (i == 0)
                {
                    map_points_.push_back(mp);
                    mp_id_.insert(mp->id_);
                }
                // 判断该点是否已被插入到优化队列中
                else if (mp_id_.find(mp->id_) != mp_id_.end())
                {
                    continue;
                }
                else
                {
                    map_points_.push_back(mp);
                    mp_id_.insert(mp->id_);
                }
            }
        }

        // 选择要固定的帧
        for (auto mp : map_points_)
        {
            for (auto iter = mp->obs_kfms_.begin(); iter != mp->obs_kfms_.end(); iter++)
            {
                Frame::Ptr fm;
                if (!iter->expired())
                    fm = iter->lock();
                else
                    continue;

                // 判断该帧是否已被插入到优化队列中
                if (fm_id_.find(fm->id_) != fm_id_.end())
                    continue;
                // 判断该帧是否已被插入到固定帧队列中
                else if (fx_id_.find(fm->id_) != fx_id_.end())
                    continue;
                else
                {
                    fixed_frames_.push_back(fm);
                    fx_id_.insert(fm->id_);
                }
            }
        }
        fm_id_.clear();
        mp_id_.clear();
        fx_id_.clear();

        // 输出
        cout << "要优化的帧数：" << frames_.size() << "  要优化的地图点数：" << map_points_.size() << "  固定的帧数：" << fixed_frames_.size() << endl;
    }

    /**
     * @brief 剔除外点
     * 
     * @param edge 图优化中的边
     * @param frame_id 边的帧顶点，及该地图点在该帧中的索引
     * @param mpoint_id 边的地图点顶点，及该帧在该地图点中的索引
     */
    void Optimization::eraseOutliers(vector<g2o::EdgeStereoSE3ProjectXYZ *> edge, vector<pair<Frame::Ptr, int>> frame_id, vector<pair<MapPoint::Ptr, int>> mpoint_id)
    {
        for (size_t i = 0; i < edge.size(); i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = edge[i];
            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                Frame::Ptr fm = frame_id[i].first;
                fm->mpoints_matched_[frame_id[i].second] = nullptr;

                MapPoint::Ptr mp = mpoint_id[i].first;
                mp->obs_kfms_id_.erase(fm);
                int count = 0;
                for (auto it = mp->obs_kfms_.begin(); it != mp->obs_kfms_.end(); it++)
                {
                    if (count == mpoint_id[i].second)
                    {
                        mp->obs_kfms_.erase(it);
                        break;
                    }
                }
            }
        }
    }

    /**
 * @brief 局部地图优化
 * 
 * @param lLocalKeyFrames 要优化的帧
 * @param lLocalMapPoints 要优化的帧所观测到的地图点
 * @param lFixedCameras 地图点所能观测到的固定的帧
 */
    void Optimization::optimiseLocalMap(vector<Frame::Ptr> lLocalFrames,
                                        vector<MapPoint::Ptr> lLocalMapPoints,
                                        vector<Frame::Ptr> lFixedFrames)
    {

        SparseOptimizer optimizer;

        std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>()); //线性方程求解器

        std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3(std::move(linearSolver))); //矩阵块求解器，求解雅克比矩阵和海森矩阵

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr)); //梯度下降方法，迭代算法

        optimizer.setAlgorithm(solver); //设置优化器

        unsigned long maxkfid = 0;

        // 加入要优化的帧
        for (auto lit = lLocalFrames.begin(), lend = lLocalFrames.end(); lit != lend; lit++)
        {
            Frame::Ptr pKfi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(
                g2o::SE3Quat((pKfi->T_wc_.inverse()).rotationMatrix(), (pKfi->T_wc_.inverse()).translation()));
            vSE3->setId(pKfi->id_);
            vSE3->setFixed(pKfi->id_ == 0);
            optimizer.addVertex(vSE3);
            if (pKfi->id_ > maxkfid)
                maxkfid = pKfi->id_;
        }

        // 加入固定的帧
        for (auto lit = lFixedFrames.begin(), lend = lFixedFrames.end(); lit != lend; lit++)
        {
            Frame::Ptr pKfi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(
                g2o::SE3Quat((pKfi->T_wc_.inverse()).rotationMatrix(), (pKfi->T_wc_.inverse()).translation()));
            vSE3->setId(pKfi->id_);
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKfi->id_ > maxkfid)
                maxkfid = pKfi->id_;
        }

        // 加入要优化的地图点
        const float thHuberStereo = sqrt(7.815);
        vector<g2o::EdgeStereoSE3ProjectXYZ *> edgestereo; // 边
        vector<pair<Frame::Ptr, int>> edgekfstereo;        // 帧顶点，及地图点在该帧的索引
        vector<pair<MapPoint::Ptr, int>> edgemapstereo;    // 地图点顶点，及帧在该地图点的索引
        for (auto lmap = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lmap != lend; lmap++)
        {
            MapPoint::Ptr pmapp = *lmap;
            g2o::VertexSBAPointXYZ *vmap = new g2o::VertexSBAPointXYZ();
            vmap->setEstimate(pmapp->coor_);
            int id = pmapp->id_ + maxkfid + 1;
            vmap->setId(id);
            vmap->setFixed(false);
            vmap->setMarginalized(true);
            optimizer.addVertex(vmap);

            int count = 0;
            for (map<weak_ptr<Frame>, int>::const_iterator kfi = pmapp->obs_kfms_id_.begin(), kfend = pmapp->obs_kfms_id_.end(); kfi != kfend; kfi++, count++)
            {
                Frame::Ptr pkf;
                if (!kfi->first.expired())
                    pkf = kfi->first.lock();
                else
                    continue;

                const Feature::Ptr ftUn = pkf->features_[kfi->second];
                if (!ftUn->stereo_)
                    continue;
                Vec3 obs;
                const float &kpur = ftUn->x_r_;
                obs << ftUn->x_, ftUn->y_, kpur;
                g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pkf->id_)));
                e->setMeasurement(obs);
                const float invSigma2 = ORBextractor::mvInvLevelSigma2[ftUn->octave_];
                e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuberStereo);
                e->fx = Camera::fx_;
                e->fy = Camera::fy_;
                e->cx = Camera::cx_;
                e->cy = Camera::cy_;
                e->bf = Camera::base_fx_;

                optimizer.addEdge(e);
                edgestereo.push_back(e);
                edgemapstereo.push_back(make_pair(pmapp, count));
                edgekfstereo.push_back(make_pair(pkf, kfi->second));
            }
        }

        // 两次优化
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        for (size_t i = 0; i < edgestereo.size(); i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = edgestereo[i];
            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
        }
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

        // 剔除误差过大的匹配
        eraseOutliers(edgestereo, edgekfstereo, edgemapstereo);

        // 更新优化后的相机位姿及地图点坐标
        for (auto lit = lLocalFrames.begin(), lend = lLocalFrames.end(); lit != lend; lit++)
        {
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(lit->get()->id_));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            lit->get()->T_wc_ = SE3(SE3quat.rotation(), SE3quat.translation());
            lit->get()->T_wc_ = lit->get()->T_wc_.inverse();
        }
        for (auto lmap = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lmap != lend; lmap++)
        {
            g2o::VertexSBAPointXYZ *point = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                lmap->get()->id_ + maxkfid + 1));
            lmap->get()->coor_ = point->estimate();
            // lmap->get()->computeAveViewAgl();
        }
    }

} // namespace myslam
