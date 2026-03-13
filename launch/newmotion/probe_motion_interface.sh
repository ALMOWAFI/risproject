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
    # Keep it short: Type + Pub/Sub lists.
    rostopic info "$t" | sed -n '1,80p'
  fi
}

# Common candidates (Franka setups vary a lot across labs).
print_info "/equilibrium_pose"
print_info "/cartesian_pose_command"
print_info "/cartesian_pose"
print_info "/command"
print_info "/command_pose"
print_info "/target_pose"

say ""
say "== Controller/MoveIt Signals (nodes/services) =="
if command -v rosnode >/dev/null 2>&1; then
  rosnode list | grep -E -i 'move_group|controller|franka|panda' || true
fi

if command -v rosservice >/dev/null 2>&1; then
  services="$(rosservice list || true)"
  echo "$services" | grep -E -i 'controller_manager|move_group' || true
  if echo "$services" | grep -qx "/controller_manager/list_controllers"; then
    say ""
    say "controller_manager/list_controllers:"
    rosservice call /controller_manager/list_controllers || true
  fi
fi

say ""
say "== Camera/TF Quick Check (optional) =="
if echo "$topics" | grep -qx "/camera/color/camera_info"; then
  cam_frame="$(rostopic echo -n1 /camera/color/camera_info/header/frame_id 2>/dev/null | tr -d '\"' | tr -d '\r' | tail -n1 || true)"
  if [ -n "$cam_frame" ]; then
    say "camera_info frame_id: $cam_frame"
    if command -v timeout >/dev/null 2>&1; then
      say "Trying TF: panda_link0 <- $cam_frame (2s)"
      timeout 2 rosrun tf tf_echo panda_link0 "$cam_frame" >/dev/null 2>&1 && say "OK: TF exists" || say "WARN: TF check failed (may be missing transform)"
    else
      say "NOTE: 'timeout' not available; skipping tf_echo probe"
    fi
  fi
fi

say ""
say "Next step:"
say "- Pick a command topic that has Subscribers > 0 and note its Type."
say "- If you have a PoseStamped command topic, we can publish the block pose to it."

