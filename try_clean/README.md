This folder is an isolated experimental area.

The intent is to try cleaner vision and player-selection ideas without touching
the current working ROS path in `src/` and `newmotion/`.

Nothing in this folder is wired into the normal build or launch flow.

Files:
- `scripts/player_selection.py`
  Standalone Python node for simple selection logic based on block-slot
  disappearance, not hand detection.
- `vision_node_no_player_selection.cpp`
  Experimental C++ vision variant intended to keep only block detection.
- `vision_node_simple_selection.cpp`
  Experimental C++ vision variant intended to use a simpler selection rule.
- `scripts/vision_node_full_python.py`
  Experimental full Python vision pipeline placeholder.

Stable working path remains:
- `src/vision_node.cpp`
- `src/game_node.cpp`
- `newmotion/motion_moveit_node.cpp`

Suggested lab runs:
- `rosrun memory_game vision_node_no_player_selection`
  Experimental C++ block detector only. No `/player_selection`.
- `rosrun memory_game player_selection.py`
  Python disappearance-based selection node to pair with block detection.
- `rosrun memory_game vision_node_simple_selection`
  C++ disappearance-based selection node. Uses `/detected_blocks`, not hand detection.
- `rosrun memory_game vision_node_full_python.py`
  Full experimental Python block detector.

Recommended order:
1. Stable block detection + `player_selection.py`
2. Stable block detection + `vision_node_simple_selection`
3. Full Python vision
