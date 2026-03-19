# Architecture - Current System

This document describes the repo as it exists now, not the original plan.

## Main Flow

```text
RGB + depth topics -> vision_node -> /detected_blocks, /player_selection
                                -> game_node -> /target_block, /game_state, /score
                                           -> one motion node
```

The game loop is:

1. Vision publishes fresh block detections.
2. Game generates a sequence.
3. Game sends the sequence to motion through `/target_block`.
4. Motion reports progress on `/motion_status`.
5. After the sequence is shown, vision publishes player selections.
6. Game compares the player input to the generated sequence.

## Nodes

### `vision_node`

Reads:
- RGB image
- aligned depth image
- camera info
- TF

Publishes:
- `/detected_blocks`
- `/player_selection`
- visualization markers
- optional debug images

Responsibilities:
- detect colored blocks
- estimate 3D block positions in `panda_link0`
- detect player hand/selection
- reject bad detections with ROI, workspace, depth, and mask logic

### `game_node`

Reads:
- `/detected_blocks`
- `/player_selection`
- `/motion_status`

Publishes:
- `/target_block`
- `/game_state`
- `/score`

Responsibilities:
- own the sequence and round logic
- wait for valid block detections before starting
- verify motion actually completed the expected targets
- compare player selections against the shown sequence

### Motion Layer

There are three motion executors in the repo.

#### `src/motion_node.cpp`

Purpose:
- demo/debug only
- publishes marker-style motion behavior for RViz/testing

Important:
- not real robot control
- does not use real detected target positions for execution

#### `newmotion/motion_hw_node.cpp`

Purpose:
- bridge from `/target_block` to a `geometry_msgs/PoseStamped` command topic

Use when:
- the robot stack already exposes a pose command interface

#### `newmotion/motion_moveit_node.cpp`

Purpose:
- bridge from `/target_block` to MoveIt plan-and-execute

Use when:
- the robot stack exposes `/move_group`
- this is the current intended Panda lab path

## Topics

- `/detected_blocks` - `memory_game/BlockArray`
- `/player_selection` - `memory_game/PlayerSelection`
- `/target_block` - `memory_game/Block`
- `/motion_status` - `std_msgs/String`
- `/game_state` - `std_msgs/String`
- `/score` - `std_msgs/Int32`

## Motion Status Contract

`game_node` now expects motion feedback during sequence display.

Common states:
- `MOVING_TO_TARGET:<block_id>`
- `AT_TARGET:<block_id>`
- `RETURNING_HOME:<block_id>`
- `MOVE_FAILED:<block_id>`
- `IDLE`

The important point is that real motion completion is no longer treated as a generic success signal. The game uses these statuses to verify what was actually shown.

## Real Robot Notes

For the Panda lab stack, the practical path is:

1. probe the available interface with `newmotion/probe_motion_interface.sh`
2. if `/move_group` is present, use `motion_moveit_node`
3. if a direct pose command topic exists instead, use `motion_hw_node`

## What Is Not True Anymore

These old assumptions should not be used as the main repo story anymore:

- that `motion_node` is the real robot executor
- that launch files are the main source of truth for how to run the system
- that the original task breakdown still matches the implemented architecture
