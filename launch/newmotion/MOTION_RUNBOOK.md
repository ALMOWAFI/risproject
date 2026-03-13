# Motion Runbook (Lab Day)

Goal: figure out what motion interface the lab Panda stack exposes, then connect our `/target_block` stream to it.

## 1) Start Lab Bringup

This depends on your lab. Typical:
- Panda bringup (driver + controllers)
- Camera bringup

## 2) Probe What Exists

From any terminal with ROS sourced:

```bash
bash launch/newmotion/probe_motion_interface.sh
```

You are looking for a topic that:
- has `Subscribers: ...` not empty
- and is a motion command interface

Common possibilities:
- Pose setpoint topic: `Type: geometry_msgs/PoseStamped` (best case)
- Trajectory interface: `Type: trajectory_msgs/JointTrajectory` or an action like `FollowJointTrajectory`
- MoveIt: node `/move_group` exists

## 3) If You Have A `PoseStamped` Command Topic

This is the fastest integration.

1. Run our nodes as usual:

```bash
rosrun memory_game vision_node
rosrun memory_game game_node _require_detected_blocks:=true
```

2. Publish the target block pose to the controller topic.

You can do this with a dedicated node (preferred) or manually to test.

Manual test (replace `CMD_TOPIC`):

```bash
rostopic pub -1 CMD_TOPIC geometry_msgs/PoseStamped \
'{header:{frame_id:"panda_link0"}, pose:{position:{x:0.4,y:0.0,z:0.4}, orientation:{w:1.0}}}'
```

If the robot moves, you have the right interface.

## 4) If You Only Have Trajectory/MoveIt

Then motion must be implemented as:
- a trajectory action client (FollowJointTrajectory), or
- a MoveIt client (plan + execute)

That is more code, but still doable once you know which interface exists.

## 5) Minimum Sanity Checks

```bash
rostopic echo -n1 /detected_blocks
rostopic echo -n1 /target_block
rostopic echo /motion_status
```

If `/target_block.position` is nonsense, game started before blocks were cached (enable `_require_detected_blocks:=true`).

