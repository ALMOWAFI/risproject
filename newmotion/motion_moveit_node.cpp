// MoveIt-backed motion node: consumes /target_block and executes robot motion via move_group.
// This is the hardware-ready alternative to src/motion_node.cpp (which is a demo/marker node).
//
// Requires MoveIt to be running on the robot PC (/move_group available).

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <memory_game/Block.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace {

void PublishStatus(ros::Publisher& pub, const std::string& s) {
  std_msgs::String msg;
  msg.data = s;
  pub.publish(msg);
}

}  // namespace

class MotionMoveItNode {
 public:
  MotionMoveItNode() : pnh_("~"), spinner_(2) {
    pnh_.param("planning_group", planning_group_, std::string("panda_arm"));
    pnh_.param("pose_frame", pose_frame_, std::string("panda_link0"));

    pnh_.param("planning_time", planning_time_, 5.0);
    pnh_.param("max_velocity_scaling", max_velocity_scaling_, 0.10);
    pnh_.param("max_acceleration_scaling", max_acceleration_scaling_, 0.10);

    pnh_.param("pointing_offset_z", pointing_offset_z_, 0.10);
    pnh_.param("return_home", return_home_, true);
    pnh_.param("use_current_state_as_home", use_current_state_as_home_, true);

    // If true, we keep the current end-effector orientation and only change XYZ.
    // This is usually the least surprising behavior for a lab setup.
    pnh_.param("keep_current_orientation", keep_current_orientation_, true);

    status_pub_ = nh_.advertise<std_msgs::String>("/motion_status", 10, true);
    target_sub_ = nh_.subscribe("/target_block", 20, &MotionMoveItNode::targetCb, this);

    spinner_.start();

    PublishStatus(status_pub_, "INIT");

    // MoveIt interface (must have /robot_description + /move_group alive).
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(planning_group_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);
    move_group_->setPoseReferenceFrame(pose_frame_);

    configureHomeTarget();
    home_pose_ = move_group_->getCurrentPose().pose;

    PublishStatus(status_pub_, "IDLE");

    ROS_INFO("motion_moveit_node ready");
    ROS_INFO("planning_group=%s", planning_group_.c_str());
    ROS_INFO("pose_frame=%s", pose_frame_.c_str());
  }

 private:
  void targetCb(const memory_game::Block::ConstPtr& msg) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      queue_.push_back(*msg);
    }

    ROS_INFO("Queued target block %d (queue=%zu)", msg->id, queueSize());

    maybeStartWorker();
  }

  size_t queueSize() {
    std::lock_guard<std::mutex> lk(mu_);
    return queue_.size();
  }

  void maybeStartWorker() {
    std::lock_guard<std::mutex> lk(mu_);
    if (worker_running_) {
      return;
    }
    worker_running_ = true;
    std::thread([this]() { this->workerLoop(); }).detach();
  }

  void workerLoop() {
    while (ros::ok()) {
      memory_game::Block next;
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (queue_.empty()) {
          worker_running_ = false;
          PublishStatus(status_pub_, "IDLE");
          return;
        }
        next = queue_.front();
        queue_.pop_front();
      }

      PublishStatus(status_pub_, "MOVING_TO_TARGET");
      if (!executeTarget(next)) {
        failAndStopQueue("Target execution failed");
        return;
      }

      PublishStatus(status_pub_, "AT_TARGET");

      if (return_home_) {
        PublishStatus(status_pub_, "RETURNING_HOME");
        if (!returnHome()) {
          failAndStopQueue("Return-home motion failed");
          return;
        }
      }
    }
  }

  void failAndStopQueue(const std::string& reason) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      queue_.clear();
      worker_running_ = false;
    }
    ROS_ERROR("%s", reason.c_str());
    PublishStatus(status_pub_, "MOVE_FAILED");
  }

  bool executeTarget(const memory_game::Block& b) {
    if (!move_group_) {
      ROS_ERROR("MoveGroupInterface not initialized");
      return false;
    }

    const std::string frame = !b.header.frame_id.empty() ? b.header.frame_id : pose_frame_;
    move_group_->setPoseReferenceFrame(frame);

    geometry_msgs::Pose target_pose;
    if (keep_current_orientation_) {
      target_pose = move_group_->getCurrentPose().pose;
    } else {
      target_pose = home_pose_;
    }

    target_pose.position = b.position;
    target_pose.position.z += pointing_offset_z_;

    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned =
        (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      ROS_WARN("Planning failed for block %d", b.id);
      move_group_->clearPoseTargets();
      return false;
    }

    const bool executed =
        (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    move_group_->stop();
    move_group_->clearPoseTargets();

    if (!executed) {
      ROS_WARN("Execution failed for block %d", b.id);
      return false;
    }

    ROS_INFO("Reached block %d", b.id);
    return true;
  }

  bool returnHome() {
    if (!move_group_) {
      return false;
    }

    // Go back to the joint configuration we started from.
    move_group_->setJointValueTarget(home_joint_values_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned =
        (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      ROS_WARN("Planning home failed");
      return false;
    }

    const bool executed =
        (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    move_group_->stop();

    return executed;
  }

  void configureHomeTarget() {
    std::vector<double> configured_home;
    const size_t joint_count = move_group_->getCurrentJointValues().size();
    if (pnh_.getParam("home_joint_values", configured_home) && !configured_home.empty()) {
      if (configured_home.size() == joint_count) {
        home_joint_values_ = configured_home;
        ROS_INFO("Using configured home_joint_values for return-home");
        return;
      }
      ROS_WARN("Ignoring home_joint_values: expected %zu joints, got %zu",
               joint_count, configured_home.size());
    }

    if (use_current_state_as_home_) {
      home_joint_values_ = move_group_->getCurrentJointValues();
      ROS_INFO("Using current robot state as home target");
    } else {
      return_home_ = false;
      ROS_ERROR("Return-home disabled: set ~home_joint_values or enable ~use_current_state_as_home");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher status_pub_;
  ros::Subscriber target_sub_;

  ros::AsyncSpinner spinner_;

  std::string planning_group_;
  std::string pose_frame_;
  double planning_time_ = 5.0;
  double max_velocity_scaling_ = 0.10;
  double max_acceleration_scaling_ = 0.10;
  double pointing_offset_z_ = 0.10;
  bool return_home_ = true;
  bool use_current_state_as_home_ = true;
  bool keep_current_orientation_ = true;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<double> home_joint_values_;
  geometry_msgs::Pose home_pose_;

  std::mutex mu_;
  std::deque<memory_game::Block> queue_;
  bool worker_running_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_moveit_node");
  MotionMoveItNode node;
  ros::waitForShutdown();
  return 0;
}
