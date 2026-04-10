#!/bin/bash
# Start the RealSense camera with aligned depth enabled.
# aligned_depth_to_color is required so depth pixels match color pixels exactly.
rosrun realsense2_camera realsense2_camera_node \
    _camera:=realsense \
    _align_depth:=true \
    _enable_sync:=true \
    _enable_color:=true \
    _enable_depth:=true
