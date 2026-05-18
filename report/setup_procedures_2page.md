# Memory Game — Setup Procedures

*Robotics and Intelligent Systems Lab 2 · Constructor University Bremen · Ali, Sinan, Izat, Boburjon*

Reproducing the demo from a fresh terminal. Total time: ~5 minutes.

## 1. Workspace Preparation

1. Mount the Franka Emika Panda on its plate at the back edge of the
   table; engage the brakes only after the cable run is clear.
2. Mount the Intel RealSense camera ~60 cm above the table on the
   support pole, angled to face the play area.
3. Place 3 colored blocks (green, blue, yellow) on the table within
   the camera's view, at least 5 cm apart, and at least 20 cm from
   the robot base.
4. Verify no large same-color objects are in frame (e.g., blue jeans,
   green folders).

> **[Photo 1: workspace layout — robot, camera, blocks on table]**

## 2. Software Startup

Open 7 terminals; source the workspace in each (`source ~/catkin_ws/devel/setup.bash`).

```bash
# T1 — ROS master
roscore

# T2 — RealSense (wait for /camera topics to appear)
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# T3 — Vision
rosparam load ~/catkin_ws/src/memory_game/config/game_params.yaml
rosrun memory_game vision_node

# T4 — Player selection
rosrun memory_game player_selection.py

# T5 — Motion (requires MoveIt move_group running)
rosrun memory_game motion_moveit_node _planning_group:=arm

# T6 — Game
rosrun memory_game game_node

# T7 — Web UI (optional)
python3 ~/catkin_ws/src/memory_game/ui/server.py
# Browser: http://127.0.0.1:8000
```

## 3. Calibration

**Image crop** — open `rqt_image_view /vision/debug_overlay`. A green
rectangle marks the active detection area. Adjust the four
`image_crop_*` values in `config/game_params.yaml` until the rectangle
wraps just the play area (hover the mouse on the overlay to read
pixel coordinates). After editing, reload params and restart vision (T3).

> **[Photo 2: rqt overlay with green crop rectangle correctly placed]**

**Color thresholds** — subscribe to `/vision/debug_mask`. Each block
should appear as a clean white blob; nothing else should. If a block is
missing or non-block objects appear, adjust the corresponding entry
under `hsv_ranges:` in `game_params.yaml` and restart vision.

> **[Photo 3: debug mask showing only the three blocks]**

## 4. Triggering the Interaction

1. Once all 3 blocks are stably detected, the game node prints
   `Starting new game`. Stay clear of the robot's workspace.
2. The robot performs a hover-point-hover motion above each block in
   the generated sequence. The UI displays `Watch the sequence`.
3. After the demonstration, the UI shows `Your turn`. Reproduce the
   sequence by **physically removing each block from its initial
   position** in the correct order. The system registers each
   selection when a block has been absent from its slot for >0.6 s.

> **[Photo 4: robot pointing at a block during the sequence]**

4. After all selections, the game compares them to the sequence,
   updates the score, and either starts a new round or shows
   `Round complete`.

## 5. Shutdown

Stop the nodes in reverse order: Ctrl+C in T7 → T6 → T5 → T4 → T3 → T2 → T1.

When stopping motion (T5), the arm halts at its current pose. Return
it to a safe configuration via Franka Desk, then engage the brakes
and power down the robot.

> **[Photo 5: Franka Desk safe-pose / brakes-engaged state]**

## Troubleshooting (Quick Reference)

| Symptom | Fix |
|---|---|
| Vision shows `NO CAMERA INFO` | T2 not running or wrong topic name |
| Vision shows `NO DEPTH SYNC` | Ensure `align_depth:=true` in T2 |
| No green rectangle on overlay | T3 not running |
| Blocks missing from `debug_mask` | Tighten/loosen HSV ranges (Section 3) |
| Game stuck on `Waiting for blocks` | Fewer than 3 blocks detected; check overlay |
| Robot doesn't move on game start | T5 not subscribed to `/target_sequence`; restart |
| Selection not triggering | Lift block fully off table for >0.6 s |
