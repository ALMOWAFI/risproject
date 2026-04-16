# Motion Runbook — memory_game robot pointer

## Overview

The robot points to blocks on the table using a physical pointer (pen, pencil, rod, etc.)
mounted as the end-effector. For each block in the sequence:

1. The arm moves (joint-space) to a **hover pose** directly above the block.
2. The arm executes a **Cartesian dip**: hover → touch-point → hover.
3. The arm moves (joint-space) to the **hover pose of the next block** — it does NOT
   return to a home pose between blocks.
4. After the last block, the arm optionally returns home (`~return_home true`).

Inputs and outputs of the motion node are unchanged:

| Direction | Topic            | Type                     |
|-----------|------------------|--------------------------|
| In        | `/target_block`  | `memory_game/Block`      |
| Out       | `/motion_status` | `std_msgs/String`        |

---

## Status messages published on `/motion_status`

| Message                  | Meaning                                              |
|--------------------------|------------------------------------------------------|
| `INIT`                   | Node starting up, MoveIt interface not ready yet     |
| `IDLE`                   | All queued blocks processed, robot stopped           |
| `MOVING_TO_TARGET:<id>`  | Joint-space move to hover pose above block `<id>`    |
| `POINTING:<id>`          | Cartesian dip in progress for block `<id>`           |
| `AT_TARGET:<id>`         | Dip complete, pointer touched block `<id>`           |
| `RETURNING_HOME`         | Final return-home move after last block              |

---

## Which node to run

### Primary: `motion_moveit_node` (MoveIt, simulation + hardware)

Requires `/move_group` to be running.

```bash
rosrun memory_game motion_moveit_node
# or via launch file:
roslaunch memory_game motion_moveit.launch
```

### Fallback: `motion_hw_node` (open-loop PoseStamped, hardware only)

Use only when MoveIt is unavailable. Does not perform Cartesian dips — publishes
a single PoseStamped per block and uses fixed timers.

```bash
rosrun memory_game motion_hw_node
```

---

## Key parameters — `motion_moveit_node`

All parameters are on the private namespace (`~`).

| Parameter                   | Default         | Description |
|-----------------------------|-----------------|-------------|
| `planning_group`            | `panda_arm`     | MoveIt planning group |
| `pose_frame`                | `panda_link0`   | Reference frame for all target poses |
| `planning_time`             | `5.0`           | Max seconds MoveIt may spend planning |
| `max_velocity_scaling`      | `0.10`          | Velocity limit (0–1); keep low during demos |
| `max_acceleration_scaling`  | `0.10`          | Acceleration limit (0–1) |
| `travel_z`                  | `0.35`          | Minimum absolute Z for hover poses (safety floor) |
| `tool_offset_z`             | `0.05`          | Z offset above `block.position.z` for the touch-point |
| `approach_margin`           | `0.10`          | Additional clearance above the block for hover poses |
| `cartesian_eef_step`        | `0.01`          | Max Cartesian step size (m) during dip |
| `cartesian_fraction_min`    | `0.90`          | Minimum fraction of path that must be planned; below this the dip is aborted |
| `return_home`               | `true`          | Return to home joints after the final block |
| `use_current_state_as_home` | `true`          | Capture current joint state at startup as the home target |

### Hover Z calculation
