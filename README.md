# Memory Game - Franka Panda + ROS1

Memory game with a Franka Emika Panda arm. Vision detects colored blocks and player selections, game logic generates and checks the sequence, and motion points the robot at the requested target.

Team: Ali, Sinan, Izat, Boburjon

## What Is Actually In This Repo

This repo now has one stable game pipeline and three motion paths:

- `vision_node`: detects blocks and player selections from RGB + depth
- `game_node`: owns sequence generation, score, round flow, and motion requests
- Demo motion: `src/motion_node.cpp`
  - RViz/debug only
  - does not use real block positions
- Pose-topic bridge: `newmotion/motion_hw_node.cpp`
  - for setups that already accept a `geometry_msgs/PoseStamped` command topic
- MoveIt bridge: `newmotion/motion_moveit_node.cpp`
  - current real-robot path for the Panda lab setup

So the real robot story is no longer just "three nodes in src". The real hardware path is:

```text
camera topics -> vision_node -> /detected_blocks, /player_selection
                             -> game_node -> /target_block
                                        -> motion_moveit_node (or motion_hw_node on other setups)
```

## Hardware

- Franka Emika Panda: robot arm
- RGB camera: color image for block/player detection
- Depth camera: 3D block position

We do not implement low-level robot or camera drivers here. We consume their ROS interfaces.

## Main Topics

- `/detected_blocks` - `memory_game/BlockArray` - published by `vision_node`
- `/player_selection` - `memory_game/PlayerSelection` - published by `vision_node`
- `/target_block` - `memory_game/Block` - published by `game_node`
- `/motion_status` - `std_msgs/String` - published by whichever motion node you run
- `/game_state` - `std_msgs/String` - published by `game_node`
- `/score` - `std_msgs/Int32` - published by `game_node`

## Motion Paths

### 1. Demo / RViz

Use when you only want to see the logic flow and markers.

```bash
rosrun memory_game motion_node
```

This node is not real robot control.

### 2. Pose-Topic Hardware Bridge

Use only if the robot stack already exposes a pose command topic.

```bash
rosrun memory_game motion_hw_node _command_pose_topic:=/your_pose_topic
```

### 3. MoveIt Hardware Bridge

Use this for the Panda lab setup that exposes `/move_group`.

```bash
rosrun memory_game motion_moveit_node
```

## Build

```bash
cd ~/catkin_ws
catkin_make --pkg memory_game
source devel/setup.bash
```

Note:
- `motion_moveit_node` is only built on machines where MoveIt is installed.
- On machines without MoveIt, the rest of the package still builds.

## Recommended Run Modes

### Vision only

```bash
rosrun memory_game vision_node
```

### Game only

```bash
rosrun memory_game game_node
```

### Real robot path in the lab

```bash
rosrun memory_game motion_moveit_node
rosrun memory_game vision_node
rosrun memory_game game_node
```

### RViz / demo path

```bash
roslaunch memory_game test_rviz.launch
```

Treat `test_rviz.launch` as a debug/demo tool, not the real robot architecture.

## Repo Layout

- `src/vision_node.cpp` - RGB/depth block detection and player selection
- `src/game_node.cpp` - game state machine, sequence, score, motion requests
- `src/motion_node.cpp` - demo marker motion only
- `newmotion/motion_hw_node.cpp` - pose-topic hardware bridge
- `newmotion/motion_moveit_node.cpp` - MoveIt hardware bridge
- `newmotion/probe_motion_interface.sh` - lab probe script for motion interfaces
- `newmotion/MOTION_RUNBOOK.md` - lab-day motion notes
- `msg/*.msg` - package messages
- `config/game_params.yaml` - runtime parameters
- `launch/test_rviz.launch` - RViz/demo launch
- `launch/vision_only.launch` - vision launch helper
- `launch/full_system.launch` - legacy convenience launch, not the recommended real-robot entrypoint

## Documentation

- `ROS1_GUIDE.md` - practical ROS commands and current workflow
- `VISION_TUNING.md` - vision calibration and debugging
- `docs/architecture.md` - current system architecture
- `docs/vision_node_cpp.md` - internal explanation of the vision implementation
- `newmotion/MOTION_RUNBOOK.md` - real-robot motion integration notes
- `TASK_BREAKDOWN.md` - historical planning document, not the current source of truth

## Integration Lessons

These are the main issues we actually hit while integrating in the lab:

- Vision initially subscribed to the wrong camera topics.
  - The working defaults were changed to the RealSense topics under `/realsense/...`.
- Vision could detect background objects as blocks.
  - ROI and workspace filtering were added to reduce false detections.
- Skin tones could overlap with the red block mask.
  - The skin mask is now removed from block masks before color detection.
- `player_selection` could keep reporting red even when the user pointed elsewhere.
  - This turned out to be downstream of bad block detection, especially false red detection from the hand.
- The game node used to keep stale block positions for too long.
  - It now treats detections more carefully and checks freshness instead of trusting old data forever.
- The game node used to fall back to fake default targets.
  - That was unsafe for real motion, so required detections are now enforced.
- The lab Panda stack did not expose the simple motion interface we first assumed.
  - The real robot path became `motion_moveit_node` after probing the available interfaces.
- The original plan and some old docs did not match the final architecture.
  - The repo now keeps the plan as historical context, not as the current source of truth.

## Current Project Status

Current practical state of the system:

- block detection is usable in the current 3-color lab path
- game logic is integrated with real detected blocks
- real motion works through `motion_moveit_node` on the Panda lab setup
- the current working MoveIt group on the lab robot is `arm`

What is still not closed:

- red is still unreliable in the real scene
- hand-based player selection is still being hardened
- UI/operator tooling is separate from the core ROS nodes and not part of the main robot path yet

So the system is no longer blocked on architecture or motion wiring. The remaining work is mainly perception reliability.

## Current Lab Commands

These are the practical commands for the current lab path:

### Motion

```bash
rosrun memory_game motion_moveit_node _planning_group:=arm
```

### Vision

```bash
rosrun memory_game vision_node \
  _disable_red:=true \
  _max_depth_age_sec:=1.0 \
  _depth_buffer_size:=10 \
  _enable_player_detection:=true \
  _selection_hold_sec:=1.0 \
  _selection_cooldown_sec:=1.0 \
  _max_select_distance_px:=90 \
  _min_hand_area:=200 \
  _require_hand_release:=false \
  _workspace_enable:=false \
  _roi_enable:=false \
  _min_block_area:=500 \
  _mask_open_iterations:=2 \
  _mask_close_iterations:=2
```

### Game

```bash
rosrun memory_game game_node \
  _disable_red:=true \
  _require_detected_blocks:=true \
  _min_detected_blocks_required:=3
```

## Troubleshooting

- Package not found:
  - `source ~/catkin_ws/devel/setup.bash`
- Message type unknown:
  - rebuild, then source again
- `motion_moveit_node` missing:
  - MoveIt is not installed in that workspace
- Vision publishes nothing:
  - check camera topics, depth, camera info, and TF to `panda_link0`
- Game waits for blocks:
  - vision is not publishing fresh valid block detections yet
