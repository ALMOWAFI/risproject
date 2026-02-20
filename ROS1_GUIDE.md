# ROS1 guide — Memory Game (Franka Panda)

Short reference for **this project**: Franka Panda arm + RGB camera + depth camera, ROS1.

---

## Our setup

| Component | What we use | ROS (we don't write these) |
|-----------|-------------|----------------------------|
| Robot | Franka Emika Panda | `franka_ros` — provides `joint_states`, we send motion |
| RGB image | One color camera | Topic e.g. `/camera/color/image_raw` |
| Depth | One depth camera | Topic e.g. `/camera/depth/image_raw` |

We only write: **vision_node**, **game_node**, **motion_node**. Robot and cameras run from their own drivers.

---

## Workspace (one-time)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
git clone <repo-url> memory_game
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Add to `~/.bashrc` so you don’t forget:

```bash
source /opt/ros/noetic/setup.bash   # or melodic
source ~/catkin_ws/devel/setup.bash
```

---

## Commands we actually use

**Build**

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Run our package**

```bash
# All nodes
roslaunch memory_game full_system.launch

# Or one node (e.g. for debugging)
rosrun memory_game vision_node
rosrun memory_game game_node
rosrun memory_game motion_node
```

**See what’s running**

```bash
rosnode list
rostopic list
rostopic echo /detected_blocks
rostopic echo /game_state
rqt_graph   # node–topic graph
```

**Our topics (for reference)**

| Topic | Type | Who publishes |
|-------|------|----------------|
| `/camera/color/image_raw` | Image | Camera driver |
| `/camera/depth/image_raw` | Image | Camera driver |
| `/detected_blocks` | BlockArray | vision_node |
| `/player_selection` | PlayerSelection | vision_node |
| `/target_block` | Block | game_node |
| `/game_state` | String | game_node |
| `/score` | Int32 | game_node |

**Debug**

```bash
# See message definition
rosmsg show memory_game/Block

# Check topic rate
rostopic hz /camera/color/image_raw
```

---

## Running with real robot + cameras

1. Start Panda (replace with your robot IP):

   ```bash
   roslaunch franka_control franka_control.launch robot_ip:=YOUR_PANDA_IP
   ```

2. Start cameras (e.g. RealSense):

   ```bash
   roslaunch realsense2_camera rs_camera.launch
   ```

3. Start our game:

   ```bash
   roslaunch memory_game full_system.launch
   ```

(If you use different camera/robot launch files at the lab, swap the first two steps with those.)

---

## Simulation (Gazebo)

When the sim launch file is set up:

```bash
roslaunch memory_game simulation.launch
```

Then in another terminal:

```bash
roslaunch memory_game full_system.launch
```

---

## Fixes

| Problem | Try |
|--------|-----|
| Package not found | `source ~/catkin_ws/devel/setup.bash` |
| Message type unknown | `catkin_make` then `source devel/setup.bash` |
| Build broken | `cd ~/catkin_ws && rm -rf build devel && catkin_make` |

---

For full ROS install (Melodic/Noetic), see the official ROS wiki. This file is only for **our** project and workflow.
