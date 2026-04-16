#!/usr/bin/env bash
set -euo pipefail
say() { printf "%s\n" "$*"; }
if ! command -v rostopic >/dev/null 2>&1; then
  say "ERROR: rostopic not found. Did you source ROS? e.g. 'source /opt/ros/noetic/setup.bash'"
  exit 1
fi
say "== ROS Master Check =="
if ! rostopic list >/dev/null 2>&1; then
  say "ERROR: cannot talk to ROS master."
  say "- Start roscore (or set ROS_MASTER_URI to the lab master)."
  exit 2
fi
say "OK: rostopic list works"
say ""
say "== Candidate Motion Interfaces (topics) =="
topics="$(rostopic list || true)"
echo "$topics" | grep -E -i 'equilibrium|cartesian|pose|command|target|trajectory|follow_joint|joint_trajectory|move_group' || true
say ""
say "== Topic Details (type + subscribers) =="
print_info() {
  local t="$1"
  if echo "$topics" | grep -qx "$t"; then
    say ""
    say "Topic: $t"
    rostopic info "$t" | sed -n '1,80p'
  fi
}
print_info "/equilibrium_pose"
print_info "/cartesian_pose_command"
print_info "/cartesian_pose"
print_info "/command"
print_info "/command_pose"
print_info "/target_pose"
print_info "/position_joint_trajectory_controller/command"
print_info "/position_joint_trajectory_controller/state"
print_info "/position_joint_trajectory_controller/follow_joint_trajectory/goal"
print_info "/position_joint_trajectory_controller/follow_joint_trajectory/status"
print_info "/position_joint_trajectory_controller/follow_joint_trajectory/feedback"
print_info "/position_joint_trajectory_controller/follow_joint_trajectory/result"
say ""
say "== Controller/MoveIt Signals (nodes/services) =="
if command -v rosnode >/dev/null 2>&1; then
  rosnode list | grep -E -i 'move_group|controller|franka|panda' || true
fi
if command -v rosservice >/dev/null 2>&1; then
  services="$(rosservice call /controller_manager/list_controllers || true)"
  echo "$services" | grep -E -i 'controller_manager|move_group' || true
  if echo "$services" | grep -qx "/controller_manager/list_controllers"; then
    say ""
    say "controller_manager/list_controllers:"
    rosservice call /controller_manager/list_controllers || true
  fi
fi
say ""
say "== Camera/TF Quick Check (optional) =="
if echo "$topics" | grep -qx "/realsense/color/camera_info"; then
  cam_frame="$(rostopic echo -n1 /realsense/color/camera_info 2>/dev/null \
    | grep frame_id | head -n1 | awk -F\" '{print $2}' || true)"
  if [ -n "$cam_frame" ]; then
    say "camera_info frame_id: $cam_frame"
    if command -v timeout >/dev/null 2>&1; then
      say "Trying TF: panda_link0 <- $cam_frame (2s)"
      timeout 2 rosrun tf tf_echo panda_link0 "$cam_frame" >/dev/null 2>&1 \
        && say "OK: TF exists" \
        || say "WARN: TF check failed (may be missing transform)"
    else
      say "NOTE: 'timeout' not available; skipping tf_echo probe"
    fi
  fi
fi
say ""
say "Next step:"
say "- Confirm /move_group is running (see nodes list above)."
say "- The primary motion node is motion_moveit_node (MoveIt, joint-move + Cartesian dip)."
say "- If /move_group is NOT running, fall back to motion_hw_node (PoseStamped command topic)."
say "- Pick a PoseStamped command topic with Subscribers > 0 and set ~command_pose_topic in the hw node launch."
