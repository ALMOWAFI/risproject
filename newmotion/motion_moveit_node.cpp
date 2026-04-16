
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <memory_game/Block.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace {
void PublishStatus(ros::Publisher& pub, const std::string& s, int id = -1) {
  std_msgs::String msg;
  if (id >= 0) {
    std::ostringstream oss;
    oss << s << ":" << id;
    msg.data = oss.str();
  } else {
    msg.data = s;
  }
  pub.publish(msg);
}
}

class MotionMoveItNode {
public:
  MotionMoveItNode() : pnh_("~"), spinner_(2) {

    // Params
    pnh_.param("planning_group", planning_group_, std::string("panda_arm"));
    pnh_.param("pose_frame", pose_frame_, std::string("panda_link0"));
    pnh_.param("planning_time", planning_time_, 5.0);
    pnh_.param("max_velocity_scaling", max_velocity_scaling_, 0.10);
    pnh_.param("max_acceleration_scaling", max_acceleration_scaling_, 0.10);
    pnh_.param("cartesian_eef_step", cartesian_eef_step_, 0.01);
    pnh_.param("cartesian_fraction_min", cartesian_fraction_min_, 0.90);
    pnh_.param("travel_z", travel_z_, 0.35);
    pnh_.param("tool_offset_z", tool_offset_z_, 0.05);
    pnh_.param("approach_margin", approach_margin_, 0.10);
    pnh_.param("return_home", return_home_, true);
    pnh_.param("use_current_state_as_home", use_current_state_as_home_, true);

    status_pub_ = nh_.advertise<std_msgs::String>("/motion_status", 10, true);
    target_sub_ = nh_.subscribe("/target_block", 20, &MotionMoveItNode::targetCb, this);

    spinner_.start();
    PublishStatus(status_pub_, "INIT");

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(planning_group_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);
    move_group_->setPoseReferenceFrame(pose_frame_);

    configureHome();

    worker_thread_ = std::thread([this]() { workerLoop(); });

    PublishStatus(status_pub_, "IDLE");
  }

  ~MotionMoveItNode() {
    {
      std::lock_guard<std::mutex> lk(mu_);
      shutting_down_ = true;
      pending_.clear();
    }
    cv_.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
  }

private:

  // ---------------- VALIDATION ----------------
  bool validateBlock(const memory_game::Block& b) const {
    return std::isfinite(b.position.x) &&
           std::isfinite(b.position.y) &&
           std::isfinite(b.position.z);
  }

  // ---------------- QUEUE ----------------
  void targetCb(const memory_game::Block::ConstPtr& msg) {
    if (!msg) return;
    std::lock_guard<std::mutex> lk(mu_);
    if (shutting_down_) return;
    pending_.push_back(*msg);
    cv_.notify_one();
  }

  void workerLoop() {
    while (true) {
      std::vector<memory_game::Block> batch;

      {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [this]() { return shutting_down_ || !pending_.empty(); });
        if (shutting_down_) return;

        batch.assign(pending_.begin(), pending_.end());
        pending_.clear();
      }

      executeBatch(batch);

      {
        std::lock_guard<std::mutex> lk(mu_);
        if (pending_.empty()) PublishStatus(status_pub_, "IDLE");
      }
    }
  }

  // ---------------- CORE ----------------
  void executeBatch(const std::vector<memory_game::Block>& batch) {
    for (const auto& b : batch) {

      if (!validateBlock(b)) {
        ROS_WARN("Invalid block %d skipped", b.id);
        continue;
      }

      double safe_z = std::max(travel_z_, b.position.z + approach_margin_);

      geometry_msgs::Pose hover = buildPose(b.position.x, b.position.y, safe_z);
      geometry_msgs::Pose point = buildPose(b.position.x, b.position.y,
                                            b.position.z + tool_offset_z_);

      PublishStatus(status_pub_, "MOVING_TO_TARGET", b.id);
      if (!jointMove(hover)) return;

      PublishStatus(status_pub_, "POINTING", b.id);
      if (!cartesianDip(hover, point)) return;

      PublishStatus(status_pub_, "AT_TARGET", b.id);
    }

    if (return_home_) {
      PublishStatus(status_pub_, "RETURNING_HOME");
      returnHome();
    }
  }

  // ---------------- MOTION ----------------
  bool jointMove(const geometry_msgs::Pose& target) {
    move_group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      return false;

    bool ok = move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    move_group_->stop();
    move_group_->clearPoseTargets();
    return ok;
  }

  bool cartesianDip(const geometry_msgs::Pose& hover,
                    const geometry_msgs::Pose& point) {

    std::vector<geometry_msgs::Pose> waypoints = {hover, point, hover};

    moveit_msgs::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(
        waypoints, cartesian_eef_step_, 0.0, traj);

    if (fraction < cartesian_fraction_min_) {
      ROS_WARN("Cartesian failed %.2f", fraction);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = traj;

    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      return false;

    move_group_->stop();

    // dwell time
    ros::Duration(0.5).sleep();

    return true;
  }

  bool returnHome() {
    if (home_joint_values_.empty()) return false;

    move_group_->setJointValueTarget(home_joint_values_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      return false;

    bool ok = move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    move_group_->stop();
    return ok;
  }

  // ---------------- POSE ----------------
  geometry_msgs::Pose buildPose(double x, double y, double z) {
    geometry_msgs::Pose p;

    // use current orientation (more stable than cached)
    p.orientation = move_group_->getCurrentPose().pose.orientation;

    p.position.x = x;
    p.position.y = y;
    p.position.z = z;

    return p;
  }

  // ---------------- HOME ----------------
  void configureHome() {
    if (use_current_state_as_home_) {
      home_joint_values_ = move_group_->getCurrentJointValues();
    }
  }

  // ---------------- MEMBERS ----------------
  ros::NodeHandle nh_, pnh_;
  ros::Publisher status_pub_;
  ros::Subscriber target_sub_;
  ros::AsyncSpinner spinner_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<double> home_joint_values_;

  std::string planning_group_, pose_frame_;

  double planning_time_;
  double max_velocity_scaling_;
  double max_acceleration_scaling_;
  double cartesian_eef_step_;
  double cartesian_fraction_min_;
  double travel_z_;
  double tool_offset_z_;
  double approach_margin_;

  bool return_home_;
  bool use_current_state_as_home_;

  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<memory_game::Block> pending_;
  std::thread worker_thread_;
  bool shutting_down_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_moveit_node");
  MotionMoveItNode node;
  ros::waitForShutdown();
  return 0;
}

