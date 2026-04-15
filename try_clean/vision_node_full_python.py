#!/usr/bin/env python3

import math

import cv2
import image_geometry
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from memory_game.msg import Block, BlockArray


def make_color(r, g, b, a=1.0):
    msg = ColorRGBA()
    msg.r = r
    msg.g = g
    msg.b = b
    msg.a = a
    return msg


def marker_color(name):
    if name == "green":
        return make_color(0.0, 1.0, 0.0)
    if name == "blue":
        return make_color(0.0, 0.2, 1.0)
    if name == "yellow":
        return make_color(1.0, 1.0, 0.0)
    return make_color(0.7, 0.7, 0.7)


class VisionNodeFullPython:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.color_topic = rospy.get_param("~color_topic", "/realsense/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/realsense/aligned_depth_to_color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/realsense/color/camera_info")
        self.target_frame = rospy.get_param("~target_frame", "panda_link0")
        self.markers_topic = rospy.get_param("~markers_topic", "/vision_python/visualization_marker_array")
        self.disable_red = rospy.get_param("~disable_red", True)
        self.min_block_area = float(rospy.get_param("~min_block_area", 500.0))
        self.max_block_area = float(rospy.get_param("~max_block_area", 60000.0))
        self.mask_open_iterations = int(rospy.get_param("~mask_open_iterations", 2))
        self.mask_close_iterations = int(rospy.get_param("~mask_close_iterations", 2))
        self.depth_window_radius = int(rospy.get_param("~depth_window_radius", 2))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.10))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 2.50))
        self.max_depth_age_sec = float(rospy.get_param("~max_depth_age_sec", 1.0))
        self.enable_debug_images = rospy.get_param("~enable_debug_images", True)
        self.roi_enable = rospy.get_param("~roi_enable", False)
        self.roi_x = int(rospy.get_param("~roi_x", 0))
        self.roi_y = int(rospy.get_param("~roi_y", 0))
        self.roi_w = int(rospy.get_param("~roi_w", 0))
        self.roi_h = int(rospy.get_param("~roi_h", 0))
        self.workspace_enable = rospy.get_param("~workspace_enable", False)
        self.workspace_min_x = float(rospy.get_param("~workspace_min_x", -10.0))
        self.workspace_max_x = float(rospy.get_param("~workspace_max_x", 10.0))
        self.workspace_min_y = float(rospy.get_param("~workspace_min_y", -10.0))
        self.workspace_max_y = float(rospy.get_param("~workspace_max_y", 10.0))
        self.workspace_min_z = float(rospy.get_param("~workspace_min_z", -10.0))
        self.workspace_max_z = float(rospy.get_param("~workspace_max_z", 10.0))
        self.marker_size_m = float(rospy.get_param("~marker_size_m", 0.05))

        self.color_configs = [
            (0, "red", [(0, 15, 60, 255, 30, 255), (165, 180, 60, 255, 30, 255)]),
            (1, "green", [(40, 85, 70, 255, 50, 255)]),
            (2, "blue", [(95, 135, 70, 255, 50, 255)]),
            (3, "yellow", [(20, 35, 100, 255, 80, 255)]),
        ]
        if self.disable_red:
            self.color_configs = [cfg for cfg in self.color_configs if cfg[0] != 0]

        self.latest_depth = None
        self.latest_depth_encoding = ""
        self.latest_depth_stamp = rospy.Time(0)
        self.has_camera_model = False

        self.blocks_pub = rospy.Publisher("/detected_blocks", BlockArray, queue_size=10)
        self.markers_pub = rospy.Publisher(self.markers_topic, MarkerArray, queue_size=10)
        self.debug_mask_pub = rospy.Publisher("/vision_python/debug_mask", Image, queue_size=1)
        self.debug_overlay_pub = rospy.Publisher("/vision_python/debug_overlay", Image, queue_size=1)

        rospy.Subscriber(self.color_topic, Image, self.color_callback, queue_size=5)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=10)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=5)

        rospy.loginfo("vision_node_full_python ready")

    def camera_info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.has_camera_model = True

    def depth_callback(self, msg):
        try:
            if msg.encoding in ("16UC1", "mono16"):
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                self.latest_depth_encoding = "16UC1"
            elif msg.encoding == "32FC1":
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
                self.latest_depth_encoding = "32FC1"
            else:
                rospy.logwarn_throttle(2.0, "Unsupported depth encoding: %s", msg.encoding)
                return
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Depth cv_bridge error: %s", exc)
            return

        self.latest_depth = depth
        self.latest_depth_stamp = msg.header.stamp

    def color_callback(self, msg):
        if not self.has_camera_model:
            rospy.logwarn_throttle(2.0, "Waiting for camera_info before processing RGB frames")
            return

        if self.latest_depth is None:
            rospy.logwarn_throttle(2.0, "Waiting for depth before processing RGB frames")
            return

        age = abs((msg.header.stamp - self.latest_depth_stamp).to_sec())
        if age > self.max_depth_age_sec:
            rospy.logwarn_throttle(2.0, "Depth too old for RGB frame (age=%.3f)", age)
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Color cv_bridge error: %s", exc)
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        debug_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        debug_overlay = bgr.copy()

        blocks = []
        for block_id, name, ranges in self.color_configs:
            det = self.detect_color_blob(hsv, ranges)
            if det is None:
                continue

            u, v, area, mask = det
            debug_mask = cv2.bitwise_or(debug_mask, mask)

            depth_m = self.read_depth_meters(u, v)
            if depth_m is None:
                continue

            base_point = self.project_to_base(u, v, depth_m, msg.header)
            if base_point is None:
                continue

            if self.workspace_enable and not self.within_workspace(base_point):
                continue

            block = Block()
            block.header.stamp = msg.header.stamp
            block.header.frame_id = self.target_frame
            block.id = block_id
            block.color = name
            block.position = base_point
            block.orientation.w = 1.0
            block.confidence = min(1.0, area / max(1.0, self.min_block_area * 8.0))
            block.is_selected = False
            blocks.append(block)

            cv2.circle(debug_overlay, (u, v), 6, (255, 255, 255), 2)
            cv2.putText(debug_overlay, name, (u + 8, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        blocks.sort(key=lambda block: block.id)
        self.publish_blocks(blocks, msg.header.stamp)
        self.publish_markers(blocks, msg.header.stamp)

        if self.enable_debug_images:
            self.publish_debug_image(self.debug_mask_pub, debug_mask, msg.header, "mono8")
            self.publish_debug_image(self.debug_overlay_pub, debug_overlay, msg.header, "bgr8")

    def detect_color_blob(self, hsv, ranges):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for h_min, h_max, s_min, s_max, v_min, v_max in ranges:
            partial = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
            mask = cv2.bitwise_or(mask, partial)

        if self.mask_open_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, iterations=self.mask_open_iterations)
        if self.mask_close_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None, iterations=self.mask_close_iterations)

        self.apply_roi(mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_block_area or area > self.max_block_area:
                continue
            if area > best_area:
                moments = cv2.moments(contour)
                if moments["m00"] <= 1e-6:
                    continue
                best_area = area
                best = (
                    int(moments["m10"] / moments["m00"]),
                    int(moments["m01"] / moments["m00"]),
                    area,
                    mask,
                )
        return best

    def apply_roi(self, mask):
        if not self.roi_enable or self.roi_w <= 0 or self.roi_h <= 0:
            return
        x1 = max(0, self.roi_x)
        y1 = max(0, self.roi_y)
        x2 = min(mask.shape[1], x1 + self.roi_w)
        y2 = min(mask.shape[0], y1 + self.roi_h)
        restricted = np.zeros_like(mask)
        restricted[y1:y2, x1:x2] = mask[y1:y2, x1:x2]
        mask[:, :] = restricted

    def read_depth_meters(self, u, v):
        if self.latest_depth is None:
            return None

        values = []
        for dv in range(-self.depth_window_radius, self.depth_window_radius + 1):
            for du in range(-self.depth_window_radius, self.depth_window_radius + 1):
                uu = u + du
                vv = v + dv
                if uu < 0 or vv < 0 or uu >= self.latest_depth.shape[1] or vv >= self.latest_depth.shape[0]:
                    continue

                if self.latest_depth_encoding == "16UC1":
                    raw = int(self.latest_depth[vv, uu])
                    if raw == 0:
                        continue
                    z = raw * 0.001
                elif self.latest_depth_encoding == "32FC1":
                    raw = float(self.latest_depth[vv, uu])
                    if not math.isfinite(raw) or raw <= 0.0:
                        continue
                    z = raw
                else:
                    return None

                if self.min_depth_m <= z <= self.max_depth_m:
                    values.append(z)

        if not values:
            return None

        values.sort()
        return values[len(values) // 2]

    def project_to_base(self, u, v, depth_m, header):
        ray = self.camera_model.projectPixelTo3dRay((float(u), float(v)))
        point = PointStamped()
        point.header = header
        point.point.x = ray[0] * depth_m
        point.point.y = ray[1] * depth_m
        point.point.z = ray[2] * depth_m

        try:
            transformed = self.tf_buffer.transform(point, self.target_frame, rospy.Duration(0.03))
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "TF transform failed: %s", exc)
            return None

        return transformed.point

    def within_workspace(self, point):
        return (
            self.workspace_min_x <= point.x <= self.workspace_max_x and
            self.workspace_min_y <= point.y <= self.workspace_max_y and
            self.workspace_min_z <= point.z <= self.workspace_max_z
        )

    def publish_blocks(self, blocks, stamp):
        msg = BlockArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame
        msg.blocks = blocks
        self.blocks_pub.publish(msg)

    def publish_markers(self, blocks, stamp):
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.target_frame
        clear.header.stamp = stamp
        clear.ns = "blocks"
        clear.id = 0
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for block in blocks:
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = stamp
            marker.ns = "blocks"
            marker.id = block.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = block.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.marker_size_m
            marker.scale.y = self.marker_size_m
            marker.scale.z = self.marker_size_m
            marker.color = marker_color(block.color)
            array.markers.append(marker)

        self.markers_pub.publish(array)

    def publish_debug_image(self, publisher, image, header, encoding):
        if publisher.get_num_connections() == 0:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Debug cv_bridge error: %s", exc)
            return
        msg.header = header
        publisher.publish(msg)


def main():
    rospy.init_node("vision_node_full_python")
    VisionNodeFullPython()
    rospy.spin()


if __name__ == "__main__":
    main()
