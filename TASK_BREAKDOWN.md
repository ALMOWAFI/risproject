# Task Breakdown - Historical Planning Note

This file is kept only as a record of the original planning phase.

It is not the current source of truth for the repo anymore. The project changed during implementation and lab integration, especially around motion and launch workflow.

Use these files instead when you want the current picture:

- `README.md`
- `ROS1_GUIDE.md`
- `docs/architecture.md`
- `newmotion/MOTION_RUNBOOK.md`

Below this note, the old planning text is preserved for reference only.

---

This guide helps you split the project into tasks so everyone can work in parallel without conflicts.

**Team:** Ali, Sinan, Izat, Boburjon

---

## 🎯 Overall Strategy

Split by **functional areas** (vision, game logic, motion) so people work on different files and don't conflict.

---

## 📋 Phase 1: Setup & Foundation (Week 1)

### Task 1.1: Repository Setup
**Who:** Anyone (Ali suggested)  
**What:**
- Set up Git repo
- Create initial package structure
- Set up CMakeLists.txt and package.xml
- Create basic message files (Block.msg, BlockArray.msg, PlayerSelection.msg)

**Files:**
- `package.xml`
- `CMakeLists.txt`
- `msg/*.msg`

**Dependencies:** None (first task)

---

### Task 1.2: Basic Node Skeletons
**Who:** Split between team  
**What:** Create empty nodes that compile and run (just print "Hello")

**Files:**
- `src/vision_node.cpp` (Ali)
- `src/game_node.cpp` (Sinan)
- `src/motion_node.cpp` (Izat)

**Dependencies:** Task 1.1 (messages must exist)

**How to split:**
- Each person creates their node skeleton
- All nodes should compile and run (even if they do nothing)
- Test: `rosrun memory_game <node_name>` should start without errors

---

### Task 1.3: Launch Files & Config
**Who:** Boburjon (or whoever wants to)  
**What:**
- Create basic launch files
- Create config files
- Set up RViz config

**Files:**
- `launch/test_rviz.launch`
- `launch/full_system.launch`
- `config/game_params.yaml`
- `config/rviz_config.rviz`

**Dependencies:** Task 1.2 (nodes must exist)

---

## 📋 Phase 2: Core Functionality (Week 2-3)

### Task 2.1: Vision Node — Mock Blocks (Test Mode)
**Who:** Ali  
**What:** Implement test mode that publishes 4 mock blocks

**Files:**
- `src/vision_node.cpp`

**What to implement:**
- Parameter `test_mode` (default: true)
- If test_mode=true: publish 4 blocks at fixed positions every second
- Publish `/detected_blocks` (BlockArray)
- Publish visualization markers for RViz

**Test:** Run `rosrun memory_game vision_node`, check `rostopic echo /detected_blocks` — should see 4 blocks

**Dependencies:** Task 1.2

---

### Task 2.2: Game Node — Sequence Generation
**Who:** Sinan  
**What:** Implement sequence generation and basic game flow

**Files:**
- `src/game_node.cpp`

**What to implement:**
- Subscribe to `/detected_blocks`
- Generate random sequence (start with length 3)
- Publish `/target_block` one by one
- Publish `/game_state` and `/score`
- Basic state machine: IDLE → SHOWING_SEQUENCE → WAITING_PLAYER

**Test:** Run game_node, should generate sequences and publish target blocks

**Dependencies:** Task 2.1 (needs blocks from vision_node)

---

### Task 2.3: Motion Node — Simulation
**Who:** Izat  
**What:** Implement simulated motion (no real robot yet)

**Files:**
- `src/motion_node.cpp`

**What to implement:**
- Subscribe to `/target_block`
- Simulate motion (just wait 2 seconds)
- Publish `/motion_status`
- Publish visualization marker (arrow) showing target
- Return to "home" after each block

**Test:** Run motion_node, when game_node publishes target_block, motion_node should log "Moving to block X"

**Dependencies:** Task 2.2 (needs target_block from game_node)

---

### Task 2.4: Integration Test
**Who:** Boburjon (or all together)  
**What:** Test all nodes together

**Files:**
- `launch/test_rviz.launch` (update if needed)

**What to do:**
- Launch all 3 nodes together
- Verify blocks appear in RViz
- Verify game generates sequences
- Verify motion node responds to targets
- Fix any connection issues

**Dependencies:** Tasks 2.1, 2.2, 2.3

---

## 📋 Phase 3: Player Input (Week 3-4)

### Task 3.1: Player Selection Detection (Mock)
**Who:** Ali  
**What:** Add mock player selection to vision_node

**Files:**
- `src/vision_node.cpp`

**What to implement:**
- In test mode: detect "player selection" (e.g. when a block is clicked in RViz, or keyboard input)
- Publish `/player_selection` when detected
- For now: can use a simple script or RViz interaction

**Test:** Publish `/player_selection` manually, verify game_node receives it

**Dependencies:** Task 2.2 (game_node must be ready to receive selections)

---

### Task 3.2: Game Logic — Input Validation
**Who:** Sinan  
**What:** Implement sequence checking

**Files:**
- `src/game_node.cpp`

**What to implement:**
- Subscribe to `/player_selection`
- Collect player input sequence
- Compare with generated sequence
- If correct: increase score, next round
- If wrong: game over

**Test:** Simulate player selections, verify game checks correctly

**Dependencies:** Task 3.1 (needs player_selection from vision_node)

---

### Task 3.3: Test Script for Player Input
**Who:** Boburjon  
**What:** Create helper script to simulate player

**Files:**
- `scripts/test_player_selection.py`

**What to implement:**
- Script that publishes `/player_selection` messages
- User types block ID (0-3)
- Publishes selection message

**Test:** Run script, type block IDs, verify game receives them

**Dependencies:** Task 3.2 (game_node must handle selections)

---

## 📋 Phase 4: Real Hardware Integration (Week 4-5)

### Task 4.1: Real Vision Detection
**Who:** Ali  
**What:** Implement real block detection from cameras

**Files:**
- `src/vision_node.cpp`

**What to implement:**
- When test_mode=false: subscribe to camera topics
- Detect blocks from RGB image (color-based)
- Get 3D positions from depth image
- Transform to robot frame
- Publish detected blocks

**Dependencies:** Task 2.1 (test mode should still work), real cameras

---

### Task 4.2: Real Player Detection
**Who:** Ali  
**What:** Detect player interactions from cameras

**Files:**
- `src/vision_node.cpp`

**What to implement:**
- Detect hand/pointing gestures
- Determine which block player selected
- Publish `/player_selection`

**Dependencies:** Task 4.1 (needs block detection first)

---

### Task 4.3: Real Robot Motion
**Who:** Izat  
**What:** Implement real Panda control

**Files:**
- `src/motion_node.cpp`

**What to implement:**
- Connect to Franka Panda (via franka_ros)
- Convert block position to robot pose
- Plan motion (using MoveIt or Franka API)
- Execute motion to point at block
- Return to home position

**Dependencies:** Task 2.3 (simulation should still work), real robot

---

## 📋 Phase 5: Polish & Testing (Week 5-6)

### Task 5.1: Error Handling
**Who:** All (each in their node)  
**What:** Add error handling and edge cases

**Files:**
- `src/vision_node.cpp` (Ali - handle camera failures, missing blocks)
- `src/game_node.cpp` (Sinan - handle missing blocks, timeouts)
- `src/motion_node.cpp` (Izat - handle robot errors, motion failures)

**What to implement:**
- Handle missing blocks gracefully
- Handle camera failures (vision_node)
- Handle robot errors (motion_node)
- Timeout handling (game_node)
- Better logging

---

### Task 5.2: Parameter Tuning
**Who:** Boburjon (or Ali)  
**What:** Tune game parameters

**Files:**
- `config/game_params.yaml`

**What to do:**
- Adjust sequence length progression
- Adjust timeouts
- Adjust scoring
- Test different difficulty levels

---

### Task 5.3: Documentation
**Who:** All (split by area)  
**What:** Finalize documentation

**Files:**
- README.md (update with final info)
- Code comments
- Architecture docs

---

## 🎯 Suggested Task Assignment

Based on team preferences and natural fit:

| Person | Main Area | Tasks |
|--------|-----------|-------|
| **Ali** | Computer Vision | Task 2.1, 3.1, 4.1, 4.2, 5.1 (vision_node) — RGB/depth processing, block detection, player detection |
| **Sinan** | Game Logic | Task 2.2, 3.2, 5.1 (game_node) — Sequence generation, scoring, validation |
| **Izat** | Motion | Task 2.3, 4.3, 5.1 (motion_node) — Robot control, motion planning |
| **Boburjon** | Infrastructure | Task 1.3, 2.4, 3.3, 5.2, 5.3, testing — Launch files, config, testing, documentation |

---

## 🔄 Workflow Tips

### Starting a Task
1. Pull latest code: `git pull origin main`
2. Create branch: `git checkout -b feature/task-name`
3. Work on your task
4. Test: build and run your part
5. Commit: `git commit -m "Add: description"`
6. Push: `git push origin feature/task-name`

### Avoiding Conflicts
- **Work on different files** — Each person has their own node:
  - **Ali** → `vision_node.cpp` (computer vision)
  - **Sinan** → `game_node.cpp` (game logic)
  - **Izat** → `motion_node.cpp` (robot motion)
  - **Boburjon** → launch files, config, scripts
- **Communicate** — If you need to touch someone else's file, tell them first
- **Pull frequently** — Stay updated with team's changes
- **Test together** — After each phase, test integration

### Testing Together
- After Phase 2: All nodes should work together in test mode
- After Phase 3: Full game flow should work with mock input
- After Phase 4: Real hardware integration
- After Phase 5: Final testing and polish

---

## 📅 Suggested Timeline

| Week | Phase | Focus |
|------|-------|-------|
| Week 1 | Phase 1 | Setup, skeletons, basic structure |
| Week 2 | Phase 2 | Core functionality (mock/test mode) |
| Week 3 | Phase 3 | Player input (mock) |
| Week 4 | Phase 4 | Real hardware integration |
| Week 5-6 | Phase 5 | Polish, testing, documentation |

**Adjust timeline based on your schedule!**

---

## ✅ Task Checklist Template

For each task, track:

- [ ] Code written
- [ ] Compiles without errors
- [ ] Tested individually
- [ ] Tested with other nodes
- [ ] Committed to branch
- [ ] Reviewed by team
- [ ] Merged to main

---

**Remember:** Start with Phase 1, work in parallel on Phase 2 (different files), then integrate and test together!
