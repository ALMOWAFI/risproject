# ROS1 Guide - Memory Game (Current Repo Workflow)

Short project-specific reference for this repo.

## What We Actually Run

Core nodes:

- `vision_node`
- `game_node`
- one motion node, depending on the environment

Motion choices:

- `motion_node`: RViz/demo only
- `motion_hw_node`: pose-topic bridge for robot stacks that already accept a pose command topic
- `motion_moveit_node`: Panda lab path when `/move_group` is available

## Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <repo-url> memory_game
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Build

```bash
cd ~/catkin_ws
catkin_make --pkg memory_game
source devel/setup.bash
```

Notes:
- `motion_moveit_node` builds only if MoveIt is installed.
- The rest of the package builds without MoveIt.

## Useful Commands

```bash
rosnode list
rostopic list
rostopic echo /detected_blocks
rostopic echo /player_selection
rostopic echo /target_block
rostopic echo /motion_status
rostopic echo /game_state
rqt_graph
```

## Camera Topics Used By Vision

Current defaults in this repo:

- `/realsense/color/image_raw`
- `/realsense/aligned_depth_to_color/image_raw`
- `/realsense/color/camera_info`

If your lab machine uses different topics, override them with params when running `vision_node`.

## Recommended Runs

### Vision only

```bash
rosrun memory_game vision_node
```

### Game only

```bash
rosrun memory_game game_node
```

### Demo motion only

```bash
rosrun memory_game motion_node
```

### Pose-topic hardware path

```bash
rosrun memory_game motion_hw_node _command_pose_topic:=/your_pose_topic
```

### MoveIt hardware path

```bash
rosrun memory_game motion_moveit_node
```

## Panda Lab Workflow

### 1. Check what motion interface the robot stack exposes

```bash
bash newmotion/probe_motion_interface.sh
```

### 2. If the lab stack exposes `/move_group`

Use:

```bash
rosrun memory_game motion_moveit_node _planning_group:=arm
```

### 3. If the lab stack exposes a pose command topic

Use:

```bash
rosrun memory_game motion_hw_node _command_pose_topic:=/your_pose_topic
```

### 4. Run the rest of the stack

```bash
rosrun memory_game vision_node
rosrun memory_game game_node
```

## Demo / RViz

```bash
roslaunch memory_game test_rviz.launch
```

This is for visualization/debugging. It is not the real robot motion path.

## Launch Files

Current meaning:

- `test_rviz.launch`: demo/debug
- `vision_only.launch`: vision helper
- `full_system.launch`: legacy convenience launch; not the recommended real-robot entrypoint
- `real_robot.launch`: bringup include only, not the whole robot-game integration story

## Lab Gotchas We Actually Hit

- Wrong camera topic names will make `vision_node` look alive but publish nothing useful.
- If `/player_selection` keeps saying red, check `/detected_blocks` first. The selection problem was usually caused by bad block detection, not the game logic itself.
- If a removed block still appears, that usually means the scene still contains a false positive for that color, not that the node is "remembering" the old block.
- The Panda lab stack may not accept a simple pose topic. Probe first with `bash newmotion/probe_motion_interface.sh` before assuming which motion node to run.
- `motion_node` is only for demo/RViz. For real robot work, use `motion_moveit_node` or `motion_hw_node` depending on the actual lab interface.
- If `rostopic echo` says it cannot load `memory_game/...` messages, rebuild and source the workspace again.

## Current Tested Lab Commands

These are the commands that match the current practical lab setup.

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
  _selection_hold_sec:=3.0 \
  _selection_cooldown_sec:=1.0 \
  _max_select_distance_m:=0.12 \
  _min_hand_area:=500 \
  _require_hand_release:=true \
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

### Useful checks while running

```bash
rostopic echo -n1 /detected_blocks
rostopic echo /player_selection
rostopic echo /motion_status
rostopic echo /game_state
rostopic echo /score
```

## Common Problems

### Message type unknown

```bash
cd ~/catkin_ws
catkin_make --pkg memory_game
source devel/setup.bash
```

### `motion_moveit_node` not found

MoveIt is not installed in that workspace, so the target was skipped at build time.

### Vision publishes nothing

Check:

```bash
rostopic hz /realsense/color/image_raw
rostopic hz /realsense/aligned_depth_to_color/image_raw
rostopic hz /realsense/color/camera_info
```

Also confirm TF exists from the camera optical frame to `panda_link0`.

### Game keeps waiting for blocks

That means `vision_node` is not publishing fresh valid block positions yet.
