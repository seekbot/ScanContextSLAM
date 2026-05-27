#include "fast_lio_sam_sc_qn.h"

// Constructor: iSAM2 + sync + timer
FastLioSamScQn::FastLioSamScQn() : Node("fast_lio_sam_sc_qn") {
    // iSAM2
    gtsam::ISAM2Params p;
    p.relinearizeThreshold = 0.01;  // re-linearize when delta > this
    p.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(p);

    // LoopClosure (config can be filled from declare_parameter)
    LoopClosureConfig cfg;
    loop_closure_ = std::make_shared<LoopClosure>(cfg);

    // Synchronised odom + cloud
    sub_odom_.subscribe(this, "/Odometry");
    sub_pcd_.subscribe(this, "/cloud_registered");
    sync_ = std::make_shared<message_filters::Synchronizer<OdomPcdSyncPol>>(
                            OdomPcdSyncPol(10), sub_odom_, sub_pcd_);
    sync_->registerCallback(std::bind(&FastLioSamScQn::odomPcdCallback, 
                                    this, std::placeholders::_1, std::placeholders::_2));

    // Loop thread = ROS2 timer on its own callback group
    loop_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000), 
        std::bind(&FastLioSamScQn::loopTimerFunc, this));
}

// Odometry/Pointcloud callback
void FastLioSamScQn::odomPcdCallback(const OdomMsg::ConstSharedPtr& odom, 
                                    const PcMsg::ConstSharedPtr& pc) 
{
    PosePcd current(*odom, *pc, (int)keyframes_.size());

    // realtime pose = last_corrected * odom_delta (smooth, lag-free output)
    odom_delta_ = odom_delta_ * last_odom_tf_.inverse() * current.pose_eig_;
    current.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
    last_odom_tf_ = current.pose_eig_;

    // FIRST frame
    if (!is_initialized_) { 
        {
            std::lock_guard<std::mutex> lk(keyframes_mutex_);
            keyframes_.push_back(current);
        }
        auto v = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4,
                                1e-2, 1e-2, 1e-2).finished();
        {
        std::lock_guard<std::mutex> lk(graph_mutex_);
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
            0, poseEigToGtsamPose(current.pose_eig_),
            gtsam::noiseModel::Diagonal::Variances(v)));                // (1) PRIOR
        init_esti_.insert(0, poseEigToGtsamPose(current.pose_eig_));
        }
        loop_closure_->updateScancontext(current.pcd_);
        // last_corrected_pose_ = current.pose_eig_;
        is_initialized_ = true;
        return;
    }

    if (checkIfKeyframe(current, keyframes_.back())) {
        int idx = (int)keyframes_.size();
        {
            std::lock_guard<std::mutex> lk(keyframes_mutex_);
            keyframes_.push_back(current);
        }
        gtsam::Pose3 from = poseEigToGtsamPose(
            keyframes_[idx-1].pose_corrected_eig_);
        gtsam::Pose3 to = poseEigToGtsamPose(current.pose_corrected_eig_);
        auto nv = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4,
                                        1e-2, 1e-2, 1e-2).finished();
        auto odom_noise = gtsam::noiseModel::Diagonal::Variances(nv);

        {
            std::lock_guard<std::mutex> lk(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                idx-1, idx, from.between(to), odom_noise));             // (2) ODOM
            init_esti_.insert(idx, to);

            isam_handler_->update(gtsam_graph_, init_esti_);
            isam_handler_->update();
            if (loop_added_flag_)
                for (int i = 0; i < 3; ++i) isam_handler_->update();    // extra passes
            gtsam_graph_.resize(0);
            init_esti_.clear();
        }
        loop_closure_->updateScancontext(current.pcd_);

        // write corrected poses back 
        corrected_esti_ = isam_handler_->calculateEstimate();
        last_corrected_pose_ = gtsamPoseToPoseEig(
            corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size()-1));
        odom_delta_ = Eigen::Matrix4d::Identity();                      // re-anchor
        
        if (loop_added_flag_) {
            std::lock_guard<std::mutex> lk(keyframes_mutex_);
            for (size_t i = 0; i < corrected_esti_.size(); ++i)
                keyframes_[i].pose_corrected_eig_ =
                    gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
            loop_added_flag_ = false;
        }
    }
}

// Loop Timer
void FastLioSamScQn::loopTimerFunc() 
{
    if (!is_initialized_) return;
    PosePcd latest;
    {
        std::lock_guard<std::mutex> lk(keyframes_mutex_);
        if (keyframes_.empty() || keyframes_.back().processed_) return;
        keyframes_.back().processed_ = true;
        latest = keyframes_.back();     // snapshot under lock
    }

    int cand;
    std::vector<PosePcd> kfs_snapshot;
    {
        std::lock_guard<std::mutex> lk(keyframes_mutex_);
        kfs_snapshot = keyframes_;          // short copy, releases lock
    }
    cand = loop_closure_->fetchCandidateKeyframeIdx(latest, kfs_snapshot);
    if (cand < 0) return;

    auto reg = loop_closure_->performLoopClosure(latest, kfs_snapshot, cand);
    if (!reg.is_valid_) {
        RCLCPP_WARN(get_logger(), "loop rejected by GICP");
        return ;
    }

    /** IMPORTANT: take care of the order. reg.pose_btwn_eig_ maps
     *  the QUERY (latest) into the CANDIDATE's frame, so:
     */
    gtsam::Pose3 from = poseEigToGtsamPose(
        reg.pose_btwn_eig_ * latest.pose_corrected_eig_);
    gtsam::Pose3 to = poseEigToGtsamPose(
        kfs_snapshot[cand].pose_corrected_eig_);
    
    // noise scales with GICP fitness [weaker match -> looser constraint]
    auto v = (gtsam::Vector(6) << reg.score_, reg.score_, reg.score_,
                                reg.score_, reg.score_, reg.score_).finished();
    {
        std::lock_guard<std::mutex> lk(graph_mutex_);
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            latest.idx_, cand, from.between(to),
            gtsam::noiseModel::Diagonal::Variances(v)));        // (3) LOOP
    }
    loop_added_flag_ = true;
}

/** Check if keyframe: Dist + rotation gate
 *  Emit a new keyframe only when the robot has moved enough since 
 *  the previous one. Without this, one keyframe per scan would be
 *  accumulated, exploding the graph. 
 */
bool FastLioSamScQn::checkIfKeyframe(const PosePcd& cur, const PosePcd& prev) 
{
    const Eigen::Matrix4d delta = prev.pose_corrected_eig_.inverse() * cur.pose_corrected_eig_;
    const double trans = delta.block<3,1>(0,3).norm();
    const Eigen::AngleAxisd aa(delta.block<3,3>(0,0));
    const double rot = std::abs(aa.angle());

    // Tune via parameters - these are reasonable defaults for outdoor LiDAR
    constexpr double kf_dist_thr_ = 1.0;                // m
    constexpr double kf_rot_thr_  = 10.0 * M_PI/180;    // rad

    return (trans > kf_dist_thr_) || (rot > kf_rot_thr_);
}