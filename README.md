# Memory Game — Franka Panda + ROS1

Memory game with a **Franka Emika Panda** arm. The robot shows a sequence by pointing at colored blocks; the player repeats it; we check and update the score.

**Team:** Ali, Sinan, Izat, Boburjon

---

## Hardware (what we use)

| Thing | Role |
|--------|--------|
| **Franka Emika Panda** | Robotic arm — points at blocks, goes home |
| **RGB camera** | Sees block colors and player (e.g. pointing) |
| **Depth camera** | Gets 3D positions of blocks (x, y, z) |

We do **not** implement the robot or camera drivers; we use their ROS topics.

---

## The game (simple)

1. **Robot shows sequence** — Panda points at blocks one by one (e.g. red → blue → green).
2. **Robot goes home** — Arm returns to a safe home pose.
3. **Player repeats** — Human points at/touches blocks in the same order (detected by cameras).
4. **We check** — If the order matches → score goes up, next round has a longer sequence. If not → game over.

So: **show sequence → wait for player → check → score/next round.**

---

## How it's built (3 nodes)

```
RGB + Depth camera  →  vision_node   →  /detected_blocks, /player_selection
                                            ↓
game_node  ←───────────────────────────────  (reads blocks + selections)
    ↓
game_node  →  /target_block, /game_state, /score
    ↓
motion_node  →  moves Panda to point at block, then home
```

- **vision_node** — From RGB + depth: finds 4 blocks (position + color), detects when the player selects a block. Publishes `BlockArray` and `PlayerSelection`.
- **game_node** — Picks random sequence, tells motion which block to point at, waits for player input, checks order, keeps score and level.
- **motion_node** — Subscribes to `target_block`, moves Panda to point at that block, then back to home.

---

## Quick start

### Build

```bash
cd ~/catkin_ws/src
git clone <repo-url> memory_game
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Run (test mode - no hardware needed)

```bash
roslaunch memory_game test_rviz.launch
```

This will:
- Start all 3 nodes in test mode
- Open RViz with 4 colored blocks visible
- Game runs automatically

**In RViz:** 4 colored cubes (red=0, green=1, blue=2, yellow=3) + magenta arrows (robot pointing)

**In terminal:** Game generates sequences, robot "points" at blocks, waits for player input

### Test player input

In another terminal:
```bash
source ~/catkin_ws/devel/setup.bash
rosrun memory_game test_player_selection.py
```

When game says "Waiting for player input...", type block IDs (0-3).

---

## Repo layout (where to edit)

| Path | What |
|------|------|
| `src/vision_node.cpp` | Block detection + player selection from RGB/depth |
| `src/game_node.cpp` | Sequence, scoring, when to show / wait / check |
| `src/motion_node.cpp` | Panda motion: point at block, go home |
| `msg/*.msg` | Message types: Block, BlockArray, PlayerSelection |
| `launch/*.launch` | How we start sim vs real robot |
| `config/game_params.yaml` | Tuning (e.g. sequence length, timeouts) |

---

## Contributing

1. **Before editing:** read [COLLABORATION.md](COLLABORATION.md) (pull before work, use branches, test before push).
2. **ROS commands / setup:** see [ROS1_GUIDE.md](ROS1_GUIDE.md).
3. **More detail on game + topics:** see [docs/architecture.md](docs/architecture.md).

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Package not found | `source ~/catkin_ws/devel/setup.bash` |
| Build broken | `cd ~/catkin_ws && rm -rf build devel && catkin_make` |
| Messages not found | Build again then `source devel/setup.bash` |
| RViz shows nothing | Fixed Frame = `panda_link0`, enable MarkerArray and Marker displays |
| Game not starting | Wait 3-4 seconds after launch (game starts after delays) |

---

## Documentation

- **[TASK_BREAKDOWN.md](TASK_BREAKDOWN.md)** — How to divide work, task assignments, timeline
- **[COLLABORATION.md](COLLABORATION.md)** — Git workflow, how to work together safely
- **[ROS1_GUIDE.md](ROS1_GUIDE.md)** — ROS commands and setup for this project
- **[VISION_TUNING.md](VISION_TUNING.md)** — Step-by-step calibration guide for `vision_node` on real camera input
- **[docs/architecture.md](docs/architecture.md)** — System design, game logic, topics
- **[docs/vision_node_cpp.md](docs/vision_node_cpp.md)** — Internal explanation of the C++ `vision_node` pipeline
