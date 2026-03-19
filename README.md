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
