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
rosrun memory_game motion_moveit_node
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
