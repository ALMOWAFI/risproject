# Architecture — game logic and nodes

Clear picture of the game and how the three nodes fit together. Hardware: **Franka Panda** + **RGB camera** + **depth camera**.

---

## The game (step by step)

1. **Show sequence**  
   Game picks a random order of blocks (e.g. red, blue, green). For each block in that order, it tells the motion node “point at this block.” The Panda points at each block one by one, then goes home.

2. **Wait for player**  
   The player must repeat the same order by pointing at or touching the blocks. The vision node sees the blocks (RGB + depth) and detects when the player “selects” a block. It publishes each selection.

3. **Check**  
   The game node compares the sequence it showed with the sequence of player selections.  
   - If they match → add score, increase sequence length, go to step 1 for the next round.  
   - If not → game over (you can define what happens next).

So the loop is: **show sequence → wait for player input → check → (next round or game over)**.

---

## Hardware (what we have)

| What | Role in the project |
|------|----------------------|
| **Franka Emika Panda** | Moves to point at a block (from 3D position), then returns to home. We don’t write the low-level driver; we use its ROS interface. |
| **RGB camera** | Gives color image. Used to see block colors and to detect player (e.g. hand pointing). |
| **Depth camera** | Gives depth image. Used with RGB to get 3D positions (x, y, z) of blocks in the robot frame. |

We only write the three nodes below. Robot and cameras run from existing drivers and publish their topics.

---

## The three nodes

### vision_node

- **Reads:** RGB image, depth image (e.g. `/camera/color/image_raw`, `/camera/depth/image_raw`).
- **Does:** Finds the 4 blocks (color + 3D position in robot frame). Detects when the player selects a block (e.g. by pointing or touching).
- **Writes:**  
  - `/detected_blocks` (BlockArray) — all blocks, with position and color.  
  - `/player_selection` (PlayerSelection) — when the player selects a block.

### game_node

- **Reads:** `/detected_blocks`, `/player_selection`.
- **Does:**  
  - Picks random sequence of blocks.  
  - Publishes `/target_block` one by one so the Panda shows the sequence.  
  - Waits for player selections.  
  - Checks if the order of selections matches the sequence.  
  - Updates score and level; publishes `/game_state` and `/score`.
- **Writes:** `/target_block`, `/game_state`, `/score`.

### motion_node

- **Reads:** `/target_block` (which block to point at, with position).
- **Does:** Moves the Panda to point at that block, then back to home. (Optionally publishes when motion is done.)
- **Writes:** (optional) `/motion_status` or similar for “moving” / “done”.

---

## Topics (summary)

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/camera/color/image_raw` | Image | Camera driver | vision_node |
| `/camera/depth/image_raw` | Image | Camera driver | vision_node |
| `/detected_blocks` | BlockArray | vision_node | game_node |
| `/player_selection` | PlayerSelection | vision_node | game_node |
| `/target_block` | Block | game_node | motion_node |
| `/game_state` | String | game_node | (optional) |
| `/score` | Int32 | game_node | (optional) |

---

## Message types (our package)

- **Block** — one block: id, color, position (x,y,z), confidence, etc.
- **BlockArray** — list of Block (all detected blocks).
- **PlayerSelection** — which block the player selected (id, color, time, etc.).

Exact definitions are in `msg/` in the repo.

---

## Data flow in one sentence

Cameras → **vision_node** → blocks and selections → **game_node** → target block → **motion_node** → Panda moves; game_node checks selections and updates score.

This doc is the single place that ties the **game logic** (show → repeat → check) to the **Franka Panda** and the **two cameras**, and makes it easy to read and contribute.
