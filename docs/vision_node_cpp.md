# vision_node.cpp Explained

This document explains exactly how `src/vision_node.cpp` works, in execution order.

## 1. Node Responsibilities

`vision_node` converts RGB + aligned depth into robot-frame block detections.

Inputs:
- RGB image (`color_topic`)
- aligned depth image (`depth_topic`)
- camera intrinsics (`camera_info_topic`)
- TF from camera optical frame to `target_frame` (`panda_link0`)

Outputs:
- `/detected_blocks` (`memory_game/BlockArray`)
- optional debug images (`/vision/debug_mask`, `/vision/debug_overlay`)

## 2. Startup and Parameters

In `VisionNode::VisionNode()`:
- `loadParams()` loads topics + tuning values.
- Publishers are created for blocks.
- Debug publishers are enabled only when `enable_debug_images` is true.
- Subscribers are started for color, depth, and camera info.

`loadParams()` also:
- clamps invalid values (negative iterations/window, smoothing bounds)
- initializes default color IDs and HSV ranges
- optionally overrides HSV from `~hsv_ranges`

Color-ID mapping used by game logic:
- `red=0`, `green=1`, `blue=2`, `yellow=3`

## 3. Data Readiness Gates

`colorCallback()` is the main loop and has strict early-exit checks:
- camera model must be available (`camera_info` received)
- latest depth frame must exist
- depth timestamp must not be older than `max_depth_age_sec` compared to RGB

If any gate fails, the frame is skipped safely.

## 4. RGB to Candidate Detections

Per frame:
1. RGB is converted from `BGR8` to HSV.
2. For each configured color, `detectColorBlob()` runs:
   - builds mask from one or more HSV ranges
   - applies open/close morphology
   - finds contours
   - keeps the largest contour within area limits
   - computes centroid from moments

Result: 2D pixel center `(u, v)` + contour area per detected color.

## 5. Depth Robustness

`readDepthMedianMeters(u, v, depth_m)`:
- samples a `(2r+1) x (2r+1)` window around centroid
- supports both `16UC1` (mm) and `32FC1` (m)
- drops invalid values (`0`, non-finite, out-of-range)
- returns median depth, not single-pixel depth

Why median:
- more stable against holes/noise/reflections than single-point depth.

## 6. 3D Projection and TF

`projectAndTransformToBase()`:
- uses camera intrinsics (`PinholeCameraModel`) to get a 3D ray
- scales ray by depth to obtain camera-frame XYZ
- transforms XYZ into `target_frame` through TF

If TF fails, detection is skipped for that frame.

## 7. Temporal Smoothing

`applyEmaSmoothing(block_id, current)`:
- stores filtered positions per block ID
- applies EMA: `filtered = a*current + (1-a)*previous`
- `smoothing_alpha` controls jitter vs responsiveness

## 8. Published Messages

For each valid block:
- `id`, `color`, and `position` are filled
- `orientation.w = 1.0`
- confidence is area-based heuristic

Blocks are sorted by ID before publishing to keep deterministic ordering.

## 9. Visualization

Debug mode overlays:
- binary accumulated mask image
- RGB overlay with centroid + color labels

## 10. Why This Design Is Production-Safe

- Strict readiness checks prevent bad frame propagation.
- Median depth + area filtering reduce noisy outliers.
- TF transform ensures game logic receives robot-frame coordinates.
- Deterministic ordering avoids nondeterministic consumer behavior.
- Optional debug outputs support tuning without changing core logic.
