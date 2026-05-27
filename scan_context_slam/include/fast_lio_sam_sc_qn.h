#pragma once
#include "utilities.hpp"
#include "pose_pcd.hpp"
#include "loop_closure.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using OdomMsg = nav_msgs::msg::Odometry;
using PcMsg = sensor_msgs::msg::PointCloud2;
using OdomPcdSyncPol = message_filters::sync_policies::ApproximateTime<OdomMsg, PcMsg>;

// Node headers and sync policy
class FastLioSamScQn : public rclcpp::Node{
public:
    FastLioSamScQn();   // Constructor
private:
    // subs (message_filters style for sync)
    message_filters::Subscriber<OdomMsg> sub_odom_;
    message_filters::Subscriber<PcMsg> sub_pcd_;
    std::shared_ptr<message_filters::Synchronizer<OdomPcdSyncPol>> sync_;

    // state
    std::shared_ptr<gtsam::ISAM2> isam_handler_;
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    std::vector<PosePcd> keyframes_;
    std::shared_ptr<LoopClosure> loop_closure_;

    Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d last_odom_tf_ = Eigen::Matrix4d::Identity();
    bool is_initialized_ = false;
    bool loop_added_flag_ = false;

    // concurrency
    std::mutex graph_mutex_, keyframes_mutex_;
    rclcpp::TimerBase::SharedPtr loop_timer_;

    void odomPcdCallback(const OdomMsg::ConstSharedPtr& odom, const PcMsg::ConstSharedPtr& pc);
    
    void loopTimerFunc();
    bool checkIfKeyframe(const PosePcd& cur, const PosePcd& prev);
};