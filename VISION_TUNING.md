# Vision Tuning Guide (ROS1, Real Camera)

Practical runbook for calibrating `vision_node` on real RGB + depth input.

## 1) Start Conditions

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws && catkin_make --pkg memory_game
```

Required runtime conditions:
- Camera publishes RGB, aligned depth, and camera info.
- TF chain exists from camera optical frame to `panda_link0`.
- `vision_node` params are loaded from `config/game_params.yaml`.

## 2) Launch for Tuning

```bash
roslaunch memory_game vision_only.launch
```

Enable debug images in `config/game_params.yaml`:

```yaml
vision_node:
  enable_debug_images: true
```

Then relaunch `vision_only.launch`.

## 3) Pre-Flight Checks (Do This First)

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
rostopic echo -n1 /camera/color/camera_info
```

Expected:
- Non-zero image rates.
- Camera info present with valid intrinsics (`K` not all zeros).

Check TF:

```bash
rosrun tf tf_echo panda_link0 <camera_optical_frame>
```

If this fails, fix TF before tuning colors.

## 4) Observe Outputs

```bash
rostopic hz /detected_blocks
rostopic echo /detected_blocks
```

Visual tools:
- `rqt_image_view` on `/vision/debug_mask` and `/vision/debug_overlay`
- RViz fixed frame `panda_link0`
- RViz display: `MarkerArray` topic `/visualization_marker_array`

## 5) Parameter Reference

All values are under `vision_node` in `config/game_params.yaml`.

| Parameter | Effect | Typical tuning direction |
|---|---|---|
| `hsv_ranges.<color>` | Color segmentation gate | Expand slightly if misses; tighten if bleed into background |
| `min_block_area` | Rejects tiny blobs | Increase to suppress speckle/noise |
| `max_block_area` | Rejects huge blobs | Decrease if table/background is detected |
| `mask_open_iterations` | Removes isolated noise | Increase for salt noise |
| `mask_close_iterations` | Fills holes in mask | Increase if block mask is fragmented |
| `depth_window_radius` | Depth median robustness | Increase for depth holes/noisy depth |
| `min_depth_m`, `max_depth_m` | Valid depth range | Widen carefully if true objects are clipped |
| `max_depth_age_sec` | Allowed depth staleness | Increase only if sync jitter is known |
| `smoothing_alpha` | Position smoothing (EMA) | Lower for less jitter, higher for faster response |

HSV range format:

```yaml
[hmin, hmax, smin, smax, vmin, vmax]
```

Note: red often needs two hue ranges due to HSV wraparound.

## 6) Fast Calibration Loop

1. Tune one color at a time with only that block in view if possible.
2. In `/vision/debug_mask`, target block should be mostly solid white.
3. Background should stay mostly black.
4. Confirm one stable detection in `/detected_blocks`.
5. Repeat for red, green, blue, yellow.
6. Tune area and morphology only after HSV is acceptable.
7. Tune depth and smoothing last.

## 7) Acceptance Criteria

Pipeline is field-ready when all checks pass:
- All four colors are detected consistently.
- IDs are correct: `red=0`, `green=1`, `blue=2`, `yellow=3`.
- Marker positions align with physical blocks in RViz (`panda_link0`).
- False positives are rare or absent in normal lighting.
- Jitter is low enough for reliable block selection.
- `/detected_blocks` publishes at a stable rate.

## Known Project-Specific Failure Patterns

These were not hypothetical; they happened during integration:

- Wrong camera topics: node runs, but `/detected_blocks` and `/player_selection` stay empty or inconsistent.
- False environment detections: a background object can win the largest-contour test for a color.
- Hand mistaken for red: skin HSV can overlap with red if skin pixels are not excluded from block masks.
- Selection always chooses one block: usually means block detections or 3D positions are wrong before selection logic runs.
- One-frame dropouts: block detection can flicker even when the setup is mostly correct, so downstream logic should not assume every single frame is perfect.

## 8) Failure Modes and Fixes

No detections:
- HSV too strict.
- Wrong input topics.
- Missing camera info.
- Lighting too dim/uneven.

Wrong 3D positions:
- TF frame mismatch to `panda_link0`.
- Depth not aligned to color.
- Reflective surfaces or depth holes.

Flicker/jitter:
- HSV too tight at decision boundary.
- Area thresholds too aggressive.
- Smoothing too responsive (`smoothing_alpha` too high).

Frequent stale depth warnings:
- Timestamp sync issue between RGB/depth.
- Depth stream rate too low for current processing rate.

## 9) Recommended Commit Practice

When calibration is complete, commit:
- `config/game_params.yaml`
- `VISION_TUNING.md` only if workflow/docs changed

Use commit messages with environment context, for example:
- `vision: tune hsv for lab lights (March 2026)`
- `vision: adjust depth window for short-range setup`
