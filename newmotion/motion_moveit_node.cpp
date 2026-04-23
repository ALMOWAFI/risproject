// This is the hardware-ready alternative to src/motion_node.cpp (which is a demo/marker node).
//
// Requires MoveIt to be running on the robot PC (/move_group available).

#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <sstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <memory_game/Block.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

namespace {

void PublishStatus(ros::Publisher& pub, const std::string& s, int block_id = -1) {
  std_msgs::String msg;
  if (block_id >= 0) {
    std::ostringstream oss;
    oss << s << ":" << block_id;
    msg.data = oss.str();
  } else {
    msg.data = s;
  }
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
    pnh_.param("require_pose_frame_match", require_pose_frame_match_, true);
    pnh_.param("workspace_enable", workspace_enable_, false);
    pnh_.param("workspace_min_x", workspace_min_x_, -10.0);
    pnh_.param("workspace_max_x", workspace_max_x_, 10.0);
    pnh_.param("workspace_min_y", workspace_min_y_, -10.0);
    pnh_.param("workspace_max_y", workspace_max_y_, 10.0);
    pnh_.param("workspace_min_z", workspace_min_z_, -10.0);
    pnh_.param("workspace_max_z", workspace_max_z_, 10.0);

    // If true, we keep the current end-effector orientation and only change XYZ.
    // This is usually the least surprising behavior for a lab setup.
    pnh_.param("keep_current_orientation", keep_current_orientation_, true);

    status_pub_ = nh_.advertise<std_msgs::String>("/motion_status", 10, true);

    spinner_.start();

    PublishStatus(status_pub_, "INIT");

    // MoveIt interface (must have /robot_description + /move_group alive).
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(planning_group_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);
    move_group_->setPoseReferenceFrame(pose_frame_);

    configureHomeTarget();
    cacheReferencePose();

    worker_thread_ = std::thread([this]() { this->workerLoop(); });
    target_sub_ = nh_.subscribe("/target_block", 20, &MotionMoveItNode::targetCb, this);

    PublishStatus(status_pub_, "IDLE");

    ROS_INFO("motion_moveit_node ready");
    ROS_INFO("planning_group=%s", planning_group_.c_str());
    ROS_INFO("pose_frame=%s", pose_frame_.c_str());
  }

  ~MotionMoveItNode() {
    {
      std::lock_guard<std::mutex> lk(mu_);
      shutting_down_ = true;
      queue_.clear();
    }
    queue_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

 private:
  void targetCb(const memory_game::Block::ConstPtr& msg) {
    if (!msg) {
      return;
    }

    size_t queue_size = 0;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (shutting_down_) {
        return;
      }
      queue_.push_back(*msg);
      queue_size = queue_.size();
    }

    ROS_INFO("Queued target block %d (queue=%zu)", msg->id, queue_size);
    queue_cv_.notify_one();
  }

  void workerLoop() {
    while (true) {
      memory_game::Block next;
      {
        std::unique_lock<std::mutex> lk(mu_);
        queue_cv_.wait(lk, [this]() { return shutting_down_ || !queue_.empty(); });
        if (shutting_down_) {
          return;
        }
        next = queue_.front();
        queue_.pop_front();
      }

      PublishStatus(status_pub_, "MOVING_TO_TARGET", next.id);
      if (!executeTarget(next)) {
        failAndStopQueue("Target execution failed", next.id);
        continue;
      }

      PublishStatus(status_pub_, "AT_TARGET", next.id);

      if (return_home_) {
        PublishStatus(status_pub_, "RETURNING_HOME", next.id);
        if (!returnHome()) {
          failAndStopQueue("Return-home motion failed", next.id);
          continue;
        }
      }

      bool queue_empty = false;
      {
        std::lock_guard<std::mutex> lk(mu_);
        queue_empty = queue_.empty();
      }
      if (queue_empty) {
        PublishStatus(status_pub_, "IDLE");
      }
    }
  }

  void failAndStopQueue(const std::string& reason, int block_id = -1) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      queue_.clear();
    }
    ROS_ERROR("%s", reason.c_str());
    PublishStatus(status_pub_, "MOVE_FAILED", block_id);
    PublishStatus(status_pub_, "IDLE");
  }

  bool withinWorkspace(const geometry_msgs::Point& p) const {
    return p.x >= workspace_min_x_ && p.x <= workspace_max_x_ &&
           p.y >= workspace_min_y_ && p.y <= workspace_max_y_ &&
           p.z >= workspace_min_z_ && p.z <= workspace_max_z_;
  }

  bool executeTarget(const memory_game::Block& b) {
    if (!move_group_) {
      ROS_ERROR("MoveGroupInterface not initialized");
      return false;
    }

    if (!std::isfinite(b.position.x) || !std::isfinite(b.position.y) || !std::isfinite(b.position.z)) {
      ROS_ERROR("Rejecting block %d: non-finite target position [%.3f, %.3f, %.3f]",
                b.id, b.position.x, b.position.y, b.position.z);
      return false;
    }

    const std::string frame = !b.header.frame_id.empty() ? b.header.frame_id : pose_frame_;
    if (require_pose_frame_match_ && frame != pose_frame_) {
      ROS_ERROR("Rejecting block %d: expected frame %s but got %s",
                b.id, pose_frame_.c_str(), frame.c_str());
      return false;
    }

    if (workspace_enable_ && frame != pose_frame_) {
      ROS_ERROR("Rejecting block %d: workspace checks require pose frame %s but target is in %s",
                b.id, pose_frame_.c_str(), frame.c_str());
      return false;
    }

    move_group_->setPoseReferenceFrame(frame);

    geometry_msgs::Pose target_pose;
    if (keep_current_orientation_) {
      target_pose = move_group_->getCurrentPose().pose;
    } else {
      target_pose = reference_pose_;
    }

    target_pose.position = b.position;
    target_pose.position.z += pointing_offset_z_;

    if (workspace_enable_ && !withinWorkspace(target_pose.position)) {
      ROS_ERROR("Rejecting block %d: target pose [%.3f, %.3f, %.3f] is outside motion workspace",
                b.id,
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);
      return false;
    }

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

    // Go back to the configured or captured home joint configuration.
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

  void cacheReferencePose() {
    reference_pose_ = move_group_->getCurrentPose().pose;

    if (home_joint_values_.empty()) {
      ROS_WARN("Using current pose as orientation reference: no home joint target available");
      return;
    }

    auto robot_state = move_group_->getCurrentState();
    if (!robot_state) {
      ROS_WARN("Using current pose as orientation reference: no robot state available");
      return;
    }

    const moveit::core::JointModelGroup* joint_model_group =
        robot_state->getJointModelGroup(planning_group_);
    if (!joint_model_group) {
      ROS_WARN("Using current pose as orientation reference: joint model group %s not found",
               planning_group_.c_str());
      return;
    }

    std::string link_name = move_group_->getEndEffectorLink();
    if (link_name.empty()) {
      const std::vector<std::string>& link_names = move_group_->getLinkNames();
      if (!link_names.empty()) {
        link_name = link_names.back();
      }
    }

    if (link_name.empty()) {
      ROS_WARN("Using current pose as orientation reference: end-effector link is unknown");
      return;
    }

    robot_state->setJointGroupPositions(joint_model_group, home_joint_values_);
    robot_state->update();

    const Eigen::Isometry3d& link_tf = robot_state->getGlobalLinkTransform(link_name);
    const Eigen::Quaterniond q(link_tf.rotation());

    reference_pose_.position.x = link_tf.translation().x();
    reference_pose_.position.y = link_tf.translation().y();
    reference_pose_.position.z = link_tf.translation().z();
    reference_pose_.orientation.x = q.x();
    reference_pose_.orientation.y = q.y();
    reference_pose_.orientation.z = q.z();
    reference_pose_.orientation.w = q.w();

    ROS_INFO("Cached orientation reference from home target using link %s", link_name.c_str());
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
  bool require_pose_frame_match_ = true;
  bool workspace_enable_ = false;
  double workspace_min_x_ = -10.0;
  double workspace_max_x_ = 10.0;
  double workspace_min_y_ = -10.0;
  double workspace_max_y_ = 10.0;
  double workspace_min_z_ = -10.0;
  double workspace_max_z_ = 10.0;
  bool keep_current_orientation_ = true;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<double> home_joint_values_;
  geometry_msgs::Pose reference_pose_;

  std::mutex mu_;
  std::condition_variable queue_cv_;
  std::deque<memory_game::Block> queue_;
  std::thread worker_thread_;
  bool shutting_down_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_moveit_node");
  MotionMoveItNode node;
  ros::waitForShutdown();
  return 0;
}
