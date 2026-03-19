/**
 * @file vision_node.cpp
 * @brief Real camera-based vision node for colored block detection.
 *
 * This node detects colored blocks from RGB + depth streams and publishes:
 *   - /detected_blocks (memory_game/BlockArray)
 *   - /visualization_marker_array (visualization_msgs/MarkerArray)
 *   - /player_selection (memory_game/PlayerSelection) when hand is near a block
 *
 * Core pipeline:
 *   1) RGB BGR -> HSV conversion
 *   2) Per-color segmentation with configurable HSV ranges
 *   3) Contour filtering by area and centroid extraction
 *   4) Robust depth (median over a local pixel window)
 *   5) Pixel+depth -> 3D camera point via camera intrinsics
 *   6) Camera frame -> robot base frame transform via TF
 *   7) Optional EMA smoothing for stable positions
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <memory_game/Block.h>
#include <memory_game/BlockArray.h>
#include <memory_game/PlayerSelection.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace {

struct HsvRange {
    int h_min;
    int h_max;
    int s_min;
    int s_max;
    int v_min;
    int v_max;
};

struct ColorConfig {
    int id;
    std::string name;
    std::vector<HsvRange> ranges;
};

struct DetectionResult {
    bool valid = false;
    int u = 0;
    int v = 0;
    double area = 0.0;
    cv::Mat mask;
};

std_msgs::ColorRGBA MakeColor(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

std_msgs::ColorRGBA MarkerColor(const std::string& color_name) {
    if (color_name == "red") return MakeColor(1.0f, 0.0f, 0.0f);
    if (color_name == "green") return MakeColor(0.0f, 1.0f, 0.0f);
    if (color_name == "blue") return MakeColor(0.0f, 0.2f, 1.0f);
    if (color_name == "yellow") return MakeColor(1.0f, 1.0f, 0.0f);
    return MakeColor(0.7f, 0.7f, 0.7f);
}

}  // namespace

namespace memory_game_vision {

class VisionNode {
public:
    VisionNode()
        : nh_(),
          pnh_("~"),
          it_(nh_),
          tf_buffer_(),
          tf_listener_(tf_buffer_),
          has_depth_(false),
          has_camera_model_(false),
          last_selected_block_id_(-1) {
        loadParams();

        blocks_pub_ = nh_.advertise<memory_game::BlockArray>("/detected_blocks", 10);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(markers_topic_, 10);
        selection_pub_ = nh_.advertise<memory_game::PlayerSelection>("/player_selection", 10);

        if (enable_debug_images_) {
            debug_mask_pub_ = it_.advertise("/vision/debug_mask", 1);
            debug_overlay_pub_ = it_.advertise("/vision/debug_overlay", 1);
        }

        color_sub_ = nh_.subscribe(color_topic_, 5, &VisionNode::colorCallback, this);
        depth_sub_ = nh_.subscribe(depth_topic_, 5, &VisionNode::depthCallback, this);
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 5, &VisionNode::cameraInfoCallback, this);

        ROS_INFO("vision_node ready");
        ROS_INFO("color_topic=%s", color_topic_.c_str());
        ROS_INFO("depth_topic=%s", depth_topic_.c_str());
        ROS_INFO("camera_info_topic=%s", camera_info_topic_.c_str());
        ROS_INFO("target_frame=%s", target_frame_.c_str());
        ROS_INFO("markers_topic=%s", markers_topic_.c_str());
    }

private:
    void loadParams() {
        pnh_.param("color_topic", color_topic_, std::string("/realsense/color/image_raw"));
        pnh_.param("depth_topic", depth_topic_, std::string("/realsense/aligned_depth_to_color/image_raw"));
        pnh_.param("camera_info_topic", camera_info_topic_, std::string("/realsense/color/camera_info"));
        pnh_.param("target_frame", target_frame_, std::string("panda_link0"));
        pnh_.param("markers_topic", markers_topic_, std::string("/vision/visualization_marker_array"));
        pnh_.param("disable_red", disable_red_, false);

        // Optional ROI to ignore background (pixel coordinates in the color image).
        pnh_.param("roi_enable", roi_enable_, false);
        pnh_.param("roi_x", roi_x_, 0);
        pnh_.param("roi_y", roi_y_, 0);
        pnh_.param("roi_w", roi_w_, 0);
        pnh_.param("roi_h", roi_h_, 0);

        // Optional 3D workspace bounds in target_frame to reject background blobs.
        pnh_.param("workspace_enable", workspace_enable_, false);
        pnh_.param("workspace_min_x", workspace_min_x_, -10.0);
        pnh_.param("workspace_max_x", workspace_max_x_, 10.0);
        pnh_.param("workspace_min_y", workspace_min_y_, -10.0);
        pnh_.param("workspace_max_y", workspace_max_y_, 10.0);
        pnh_.param("workspace_min_z", workspace_min_z_, -10.0);
        pnh_.param("workspace_max_z", workspace_max_z_, 10.0);

        pnh_.param("min_block_area", min_block_area_, 300.0);
        pnh_.param("max_block_area", max_block_area_, 60000.0);
        pnh_.param("mask_open_iterations", mask_open_iterations_, 1);
        pnh_.param("mask_close_iterations", mask_close_iterations_, 2);

        pnh_.param("depth_window_radius", depth_window_radius_, 2);
        pnh_.param("min_depth_m", min_depth_m_, 0.10);
        pnh_.param("max_depth_m", max_depth_m_, 2.50);
        pnh_.param("max_depth_age_sec", max_depth_age_sec_, 0.25);

        pnh_.param("marker_size_m", marker_size_m_, 0.05);
        pnh_.param("enable_debug_images", enable_debug_images_, false);
        pnh_.param("smoothing_alpha", smoothing_alpha_, 0.35);

        pnh_.param("enable_player_detection", enable_player_detection_, true);
        pnh_.param("max_select_distance_m", max_select_distance_m_, 0.12);
        pnh_.param("selection_cooldown_sec", selection_cooldown_sec_, 1.0);
        pnh_.param("min_hand_area", min_hand_area_, 500.0);
        pnh_.param("max_hand_area", max_hand_area_, 200000.0);
        pnh_.param("selection_hold_sec", selection_hold_sec_, 3.0);
        pnh_.param("require_hand_release", require_hand_release_, true);
        pnh_.param("exclude_skin_from_block_masks", exclude_skin_from_block_masks_, true);
        pnh_.param("skin_mask_open_iterations", skin_mask_open_iterations_, 1);
        pnh_.param("skin_mask_close_iterations", skin_mask_close_iterations_, 1);

        if (mask_open_iterations_ < 0) mask_open_iterations_ = 0;
        if (mask_close_iterations_ < 0) mask_close_iterations_ = 0;
        if (depth_window_radius_ < 0) depth_window_radius_ = 0;
        smoothing_alpha_ = std::max(0.0, std::min(1.0, smoothing_alpha_));
        max_select_distance_m_ = std::max(0.01, max_select_distance_m_);
        selection_cooldown_sec_ = std::max(0.0, selection_cooldown_sec_);
        min_hand_area_ = std::max(50.0, min_hand_area_);
        max_hand_area_ = std::max(min_hand_area_, max_hand_area_);
        selection_hold_sec_ = std::max(0.0, selection_hold_sec_);
        skin_mask_open_iterations_ = std::max(0, skin_mask_open_iterations_);
        skin_mask_close_iterations_ = std::max(0, skin_mask_close_iterations_);


        roi_x_ = std::max(0, roi_x_);
        roi_y_ = std::max(0, roi_y_);
        roi_w_ = std::max(0, roi_w_);
        roi_h_ = std::max(0, roi_h_);

        initDefaultColors();
        applyColorFilters();
        loadHsvRangesFromParams();
    }

    void initDefaultColors() {
        color_configs_.clear();

        color_configs_.push_back(ColorConfig{
            0,
            "red",
            {
                {0, 10, 100, 255, 50, 255},
                {170, 180, 100, 255, 50, 255},
            },
        });

        color_configs_.push_back(ColorConfig{1, "green", {{40, 85, 70, 255, 50, 255}}});
        color_configs_.push_back(ColorConfig{2, "blue", {{95, 135, 70, 255, 50, 255}}});
        color_configs_.push_back(ColorConfig{3, "yellow", {{20, 35, 100, 255, 80, 255}}});
    }

    void applyColorFilters() {
        if (!disable_red_) {
            return;
        }

        color_configs_.erase(
            std::remove_if(color_configs_.begin(),
                           color_configs_.end(),
                           [](const ColorConfig& cfg) { return cfg.id == 0 || cfg.name == "red"; }),
            color_configs_.end());
    }

    void loadHsvRangesFromParams() {
        XmlRpc::XmlRpcValue hsv_cfg;
        if (!pnh_.getParam("hsv_ranges", hsv_cfg)) {
            ROS_WARN("Using built-in HSV defaults (param '~hsv_ranges' not found)");
            return;
        }

        if (hsv_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_WARN("Ignoring '~hsv_ranges': expected dictionary");
            return;
        }

        for (ColorConfig& cfg : color_configs_) {
            if (!hsv_cfg.hasMember(cfg.name)) {
                continue;
            }

            XmlRpc::XmlRpcValue ranges = hsv_cfg[cfg.name];
            if (ranges.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_WARN("Ignoring hsv_ranges.%s: expected list", cfg.name.c_str());
                continue;
            }

            std::vector<HsvRange> parsed;
            for (int i = 0; i < ranges.size(); ++i) {
                if (ranges[i].getType() != XmlRpc::XmlRpcValue::TypeArray || ranges[i].size() != 6) {
                    ROS_WARN("Ignoring hsv_ranges.%s[%d]: expected [hmin,hmax,smin,smax,vmin,vmax]",
                             cfg.name.c_str(), i);
                    continue;
                }

                bool ok = true;
                int vals[6] = {0};
                for (int j = 0; j < 6; ++j) {
                    if (ranges[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt) {
                        ok = false;
                        break;
                    }
                    vals[j] = static_cast<int>(ranges[i][j]);
                }
                if (!ok) {
                    ROS_WARN("Ignoring hsv_ranges.%s[%d]: all values must be integers", cfg.name.c_str(), i);
                    continue;
                }

                parsed.push_back(HsvRange{vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]});
            }

            if (!parsed.empty()) {
                cfg.ranges = parsed;
            }
        }
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        camera_model_.fromCameraInfo(msg);
        has_camera_model_ = true;
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || msg->encoding == "16UC1") {
                cv_bridge::CvImageConstPtr cv_ptr =
                    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                latest_depth_ = cv_ptr->image.clone();
                latest_depth_encoding_ = sensor_msgs::image_encodings::TYPE_16UC1;
                latest_depth_stamp_ = msg->header.stamp;
                has_depth_ = true;
                return;
            }

            if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 || msg->encoding == "32FC1") {
                cv_bridge::CvImageConstPtr cv_ptr =
                    cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
                latest_depth_ = cv_ptr->image.clone();
                latest_depth_encoding_ = sensor_msgs::image_encodings::TYPE_32FC1;
                latest_depth_stamp_ = msg->header.stamp;
                has_depth_ = true;
                return;
            }

            has_depth_ = false;
            ROS_WARN_THROTTLE(2.0, "Unsupported depth encoding: %s", msg->encoding.c_str());
        } catch (const cv_bridge::Exception& e) {
            has_depth_ = false;
            ROS_WARN_THROTTLE(2.0, "Depth cv_bridge error: %s", e.what());
        }
    }

    void colorCallback(const sensor_msgs::ImageConstPtr& msg) {
        // Guardrails: only process frames when intrinsics + fresh depth are available.
        if (!has_camera_model_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for camera_info before processing RGB frames");
            return;
        }

        if (!has_depth_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for depth frames");
            return;
        }

        if (std::fabs((msg->header.stamp - latest_depth_stamp_).toSec()) > max_depth_age_sec_) {
            ROS_WARN_THROTTLE(2.0, "Depth too old compared to RGB frame");
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            ROS_WARN_THROTTLE(2.0, "Color cv_bridge error: %s", e.what());
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        std::vector<memory_game::Block> blocks;
        blocks.reserve(color_configs_.size());

        cv::Mat skin_mask;
        const bool need_skin_mask = enable_player_detection_ || exclude_skin_from_block_masks_;
        if (need_skin_mask) {
            skin_mask = buildSkinMask(hsv);
            applyRoiMask(skin_mask);
        }

        cv::Mat debug_mask_accum = cv::Mat::zeros(hsv.size(), CV_8UC1);
        cv::Mat debug_overlay = cv_ptr->image.clone();

        if (enable_debug_images_) {
            cv::Rect roi;
            if (computeRoiRect(hsv.size(), roi)) {
                cv::rectangle(debug_overlay, roi, cv::Scalar(255, 255, 255), 2);
            }
        }

        for (const ColorConfig& cfg : color_configs_) {
            // Per-color 2D detection in image space.
            const DetectionResult det = detectColorBlob(hsv, cfg, skin_mask);
            if (enable_debug_images_ && !det.mask.empty()) {
                cv::bitwise_or(debug_mask_accum, det.mask, debug_mask_accum);
            }
            if (!det.valid) {
                continue;
            }

            // Convert 2D centroid to 3D camera point using robust median depth.
            double depth_m = 0.0;
            if (!readDepthMedianMeters(det.u, det.v, depth_m)) {
                continue;
            }

            // Transform camera-frame point into robot base frame for downstream nodes.
            geometry_msgs::Point p_base;
            if (!projectAndTransformToBase(det.u, det.v, depth_m, msg->header, p_base)) {
                continue;
            }

            // EMA helps reduce jitter in published block coordinates.

            if (workspace_enable_ && !withinWorkspace(p_base)) {
                continue;
            }

            // EMA helps reduce jitter in published block coordinates.
            p_base = applyEmaSmoothing(cfg.id, p_base);

            memory_game::Block block;
            block.header.stamp = msg->header.stamp;
            block.header.frame_id = target_frame_;
            block.id = cfg.id;
            block.color = cfg.name;
            block.position = p_base;
            block.orientation.w = 1.0;
            block.confidence = static_cast<float>(
                std::min(1.0, det.area / std::max(1.0, min_block_area_ * 8.0)));
            block.is_selected = false;

            blocks.push_back(block);

            if (enable_debug_images_) {
                cv::circle(debug_overlay, cv::Point(det.u, det.v), 6, cv::Scalar(255, 255, 255), 2);
                cv::putText(debug_overlay,
                            cfg.name,
                            cv::Point(det.u + 8, det.v - 8),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.5,
                            cv::Scalar(255, 255, 255),
                            1,
                            cv::LINE_AA);
            }
        }

        std::sort(blocks.begin(), blocks.end(), [](const memory_game::Block& a, const memory_game::Block& b) {
            return a.id < b.id;
        });

        publishBlocks(blocks, msg->header.stamp);
        publishMarkers(blocks, msg->header.stamp);

        if (enable_player_detection_ && !blocks.empty()) {
            geometry_msgs::Point hand_base;
            if (detectHandInBase(msg->header, skin_mask, hand_base)) {
                tryPublishPlayerSelection(blocks, hand_base, msg->header);
            } else {
                resetSelectionTrackingForNoHand();
            }
        } else if (enable_player_detection_) {
            resetSelectionTrackingForNoHand();
        }

        if (enable_debug_images_) {
            publishDebugImages(debug_mask_accum, debug_overlay, msg->header);
        }

        ROS_DEBUG_THROTTLE(1.0, "Published %zu detected blocks", blocks.size());
    }

    bool detectHandInBase(const std_msgs::Header& header,
                          const cv::Mat& skin_mask,
                          geometry_msgs::Point& hand_base) {
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(skin_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int best_idx = -1;
        double best_area = 0.0;
        for (size_t i = 0; i < contours.size(); ++i) {
            const double area = cv::contourArea(contours[i]);
            if (area < min_hand_area_ || area > max_hand_area_) {
                continue;
            }
            if (area > best_area) {
                best_area = area;
                best_idx = static_cast<int>(i);
            }
        }
        if (best_idx < 0) {
            return false;
        }

        const cv::Moments m = cv::moments(contours[best_idx]);
        if (m.m00 <= 1e-6) {
            return false;
        }
        const int u = static_cast<int>(m.m10 / m.m00);
        const int v = static_cast<int>(m.m01 / m.m00);

        double depth_m = 0.0;
        if (!readDepthMedianMeters(u, v, depth_m)) {
            return false;
        }
        return projectAndTransformToBase(u, v, depth_m, header, hand_base);
    }

    void tryPublishPlayerSelection(const std::vector<memory_game::Block>& blocks,
                                    const geometry_msgs::Point& hand_base,
                                    const std_msgs::Header& header) {
        const ros::Time now = ros::Time::now();
        const bool cooldown_ok = (now - last_selection_time_).toSec() >= selection_cooldown_sec_;

        int best_id = -1;
        double best_dist = max_select_distance_m_ + 1.0;
        std::string best_color;

        for (const memory_game::Block& b : blocks) {
            const double dx = hand_base.x - b.position.x;
            const double dy = hand_base.y - b.position.y;
            const double dz = hand_base.z - b.position.z;
            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (d < best_dist) {
                best_dist = d;
                best_id = b.id;
                best_color = b.color;
            }
        }

        if (best_id < 0 || best_dist > max_select_distance_m_) {
            stable_candidate_id_ = -1;
            stable_candidate_since_ = ros::Time(0);
            return;
        }

        if (best_id != stable_candidate_id_) {
            stable_candidate_id_ = best_id;
            stable_candidate_since_ = now;
        }

        if ((now - stable_candidate_since_).toSec() < selection_hold_sec_) {
            return;
        }

        if (!cooldown_ok) {
            return;
        }

        if (!selection_armed_) {
            return;
        }

        memory_game::PlayerSelection sel;
        sel.header = header;
        sel.header.frame_id = target_frame_;
        sel.block_id = best_id;
        sel.color = best_color;
        sel.selection_type = "pointed";
        sel.selection_time = now;
        sel.confidence = static_cast<float>(1.0 - best_dist / max_select_distance_m_);
        selection_pub_.publish(sel);

        last_selection_time_ = now;
        last_selected_block_id_ = best_id;
        stable_candidate_id_ = -1;
        stable_candidate_since_ = ros::Time(0);
        if (require_hand_release_) {
            selection_armed_ = false;
        }
        ROS_INFO_THROTTLE(0.5, "Player selection: block %d (%s)", best_id, best_color.c_str());
    }

    void resetSelectionTrackingForNoHand() {
        stable_candidate_id_ = -1;
        stable_candidate_since_ = ros::Time(0);
        if (require_hand_release_) {
            selection_armed_ = true;
        }
    }


    bool computeRoiRect(const cv::Size& size, cv::Rect& roi) const {
        if (!roi_enable_ || roi_w_ <= 0 || roi_h_ <= 0) {
            return false;
        }

        cv::Rect requested(roi_x_, roi_y_, roi_w_, roi_h_);
        cv::Rect bounds(0, 0, size.width, size.height);
        roi = requested & bounds;
        return roi.width > 0 && roi.height > 0;
    }

    void applyRoiMask(cv::Mat& mask) const {
        cv::Rect roi;
        if (!computeRoiRect(mask.size(), roi)) {
            return;
        }

        cv::Mat out = cv::Mat::zeros(mask.size(), mask.type());
        mask(roi).copyTo(out(roi));
        mask = out;
    }

    bool withinWorkspace(const geometry_msgs::Point& p) const {
        return p.x >= workspace_min_x_ && p.x <= workspace_max_x_ &&
               p.y >= workspace_min_y_ && p.y <= workspace_max_y_ &&
               p.z >= workspace_min_z_ && p.z <= workspace_max_z_;
    }

    cv::Mat buildSkinMask(const cv::Mat& hsv) const {
        cv::Mat skin_mask;
        cv::inRange(hsv, cv::Scalar(0, 40, 40), cv::Scalar(15, 255, 255), skin_mask);

        cv::Mat skin_mask2;
        cv::inRange(hsv, cv::Scalar(165, 40, 40), cv::Scalar(180, 255, 255), skin_mask2);
        cv::bitwise_or(skin_mask, skin_mask2, skin_mask);

        if (skin_mask_open_iterations_ > 0) {
            cv::morphologyEx(skin_mask, skin_mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1),
                             skin_mask_open_iterations_);
        }
        if (skin_mask_close_iterations_ > 0) {
            cv::morphologyEx(skin_mask, skin_mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1),
                             skin_mask_close_iterations_);
        }

        return skin_mask;
    }

    DetectionResult detectColorBlob(const cv::Mat& hsv,
                                    const ColorConfig& cfg,
                                    const cv::Mat& skin_mask) const {
        DetectionResult out;
        cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);

        for (const HsvRange& r : cfg.ranges) {
            cv::Mat partial;
            cv::inRange(hsv,
                        cv::Scalar(r.h_min, r.s_min, r.v_min),
                        cv::Scalar(r.h_max, r.s_max, r.v_max),
                        partial);
            cv::bitwise_or(mask, partial, mask);
        }

        if (mask_open_iterations_ > 0) {
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), mask_open_iterations_);
        }
        if (mask_close_iterations_ > 0) {
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), mask_close_iterations_);
        }

        // Keep only the largest contour in valid area bounds for this color.

        applyRoiMask(mask);

        if (exclude_skin_from_block_masks_ && !skin_mask.empty()) {
            mask.setTo(0, skin_mask);
        }

        // Keep only the largest contour in valid area bounds for this color.
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int best_idx = -1;
        double best_area = 0.0;
        for (size_t i = 0; i < contours.size(); ++i) {
            const double area = cv::contourArea(contours[i]);
            if (area < min_block_area_ || area > max_block_area_) {
                continue;
            }
            if (area > best_area) {
                best_area = area;
                best_idx = static_cast<int>(i);
            }
        }

        out.mask = mask;
        if (best_idx < 0) {
            return out;
        }

        const cv::Moments m = cv::moments(contours[best_idx]);
        if (m.m00 <= 1e-6) {
            return out;
        }

        out.valid = true;
        out.u = static_cast<int>(m.m10 / m.m00);
        out.v = static_cast<int>(m.m01 / m.m00);
        out.area = best_area;
        return out;
    }

    bool readDepthMedianMeters(int u, int v, double& depth_m) const {
        if (latest_depth_.empty()) {
            return false;
        }

        std::vector<double> depth_values;
        const int window = depth_window_radius_;
        depth_values.reserve(static_cast<size_t>((2 * window + 1) * (2 * window + 1)));

        for (int dv = -window; dv <= window; ++dv) {
            for (int du = -window; du <= window; ++du) {
                const int uu = u + du;
                const int vv = v + dv;
                if (uu < 0 || vv < 0 || uu >= latest_depth_.cols || vv >= latest_depth_.rows) {
                    continue;
                }

                double z = 0.0;
                if (latest_depth_encoding_ == sensor_msgs::image_encodings::TYPE_16UC1) {
                    const uint16_t raw = latest_depth_.at<uint16_t>(vv, uu);
                    if (raw == 0) {
                        continue;
                    }
                    z = static_cast<double>(raw) * 0.001;
                } else if (latest_depth_encoding_ == sensor_msgs::image_encodings::TYPE_32FC1) {
                    const float raw = latest_depth_.at<float>(vv, uu);
                    if (!std::isfinite(raw) || raw <= 0.0f) {
                        continue;
                    }
                    z = static_cast<double>(raw);
                } else {
                    return false;
                }

                if (z >= min_depth_m_ && z <= max_depth_m_) {
                    depth_values.push_back(z);
                }
            }
        }

        if (depth_values.empty()) {
            return false;
        }

        // Median depth is more robust than a single-pixel sample.
        std::nth_element(depth_values.begin(),
                         depth_values.begin() + depth_values.size() / 2,
                         depth_values.end());
        depth_m = depth_values[depth_values.size() / 2];
        return true;
    }

    bool projectAndTransformToBase(int u,
                                   int v,
                                   double depth_m,
                                   const std_msgs::Header& image_header,
                                   geometry_msgs::Point& out_base_point) {
        // Intrinsics projection: pixel -> normalized ray -> metric camera point.
        const cv::Point2d pixel(static_cast<double>(u), static_cast<double>(v));
        const cv::Point3d ray = camera_model_.projectPixelTo3dRay(pixel);

        geometry_msgs::PointStamped p_cam;
        p_cam.header = image_header;
        p_cam.point.x = ray.x * depth_m;
        p_cam.point.y = ray.y * depth_m;
        p_cam.point.z = ray.z * depth_m;

        geometry_msgs::PointStamped p_base;
        try {
            p_base = tf_buffer_.transform(p_cam, target_frame_, ros::Duration(0.03));
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "TF transform failed (%s -> %s): %s",
                              p_cam.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
            return false;
        }

        out_base_point = p_base.point;
        return true;
    }

    geometry_msgs::Point applyEmaSmoothing(int block_id, const geometry_msgs::Point& current) {
        auto it = filtered_positions_.find(block_id);
        if (it == filtered_positions_.end()) {
            filtered_positions_[block_id] = current;
            return current;
        }

        geometry_msgs::Point& prev = it->second;
        prev.x = smoothing_alpha_ * current.x + (1.0 - smoothing_alpha_) * prev.x;
        prev.y = smoothing_alpha_ * current.y + (1.0 - smoothing_alpha_) * prev.y;
        prev.z = smoothing_alpha_ * current.z + (1.0 - smoothing_alpha_) * prev.z;
        return prev;
    }

    void publishBlocks(const std::vector<memory_game::Block>& blocks, const ros::Time& stamp) {
        memory_game::BlockArray out;
        out.header.stamp = stamp;
        out.header.frame_id = target_frame_;
        out.blocks = blocks;
        blocks_pub_.publish(out);
    }

    void publishMarkers(const std::vector<memory_game::Block>& blocks, const ros::Time& stamp) {
        visualization_msgs::MarkerArray arr;

        // Clear old markers first so RViz reflects only current detections.
        visualization_msgs::Marker clear;
        clear.header.frame_id = target_frame_;
        clear.header.stamp = stamp;
        clear.ns = "blocks";
        clear.id = 0;
        clear.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(clear);

        for (const memory_game::Block& b : blocks) {
            visualization_msgs::Marker m;
            m.header.frame_id = target_frame_;
            m.header.stamp = stamp;
            m.ns = "blocks";
            m.id = b.id;
            m.type = visualization_msgs::Marker::CUBE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position = b.position;
            m.pose.orientation.w = 1.0;
            m.scale.x = marker_size_m_;
            m.scale.y = marker_size_m_;
            m.scale.z = marker_size_m_;
            m.color = MarkerColor(b.color);
            arr.markers.push_back(m);
        }

        markers_pub_.publish(arr);
    }

    void publishDebugImages(const cv::Mat& mask,
                            const cv::Mat& overlay,
                            const std_msgs::Header& header) {
        if (!enable_debug_images_) {
            return;
        }

        if (!mask.empty() && debug_mask_pub_) {
            cv_bridge::CvImage mask_img(header, sensor_msgs::image_encodings::MONO8, mask);
            debug_mask_pub_.publish(mask_img.toImageMsg());
        }

        if (!overlay.empty() && debug_overlay_pub_) {
            cv_bridge::CvImage overlay_img(header, sensor_msgs::image_encodings::BGR8, overlay);
            debug_overlay_pub_.publish(overlay_img.toImageMsg());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    image_transport::ImageTransport it_;

    ros::Subscriber color_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber camera_info_sub_;

    ros::Publisher blocks_pub_;
    ros::Publisher markers_pub_;
    ros::Publisher selection_pub_;

    image_transport::Publisher debug_mask_pub_;
    image_transport::Publisher debug_overlay_pub_;

    std::string color_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string target_frame_;
    std::string markers_topic_;
    bool disable_red_ = false;


    bool roi_enable_ = false;
    int roi_x_ = 0;
    int roi_y_ = 0;
    int roi_w_ = 0;
    int roi_h_ = 0;

    bool workspace_enable_ = false;
    double workspace_min_x_ = -10.0;
    double workspace_max_x_ = 10.0;
    double workspace_min_y_ = -10.0;
    double workspace_max_y_ = 10.0;
    double workspace_min_z_ = -10.0;
    double workspace_max_z_ = 10.0;

    double min_block_area_;
    double max_block_area_;
    int mask_open_iterations_;
    int mask_close_iterations_;

    int depth_window_radius_;
    double min_depth_m_;
    double max_depth_m_;
    double max_depth_age_sec_;

    double marker_size_m_;
    bool enable_debug_images_;
    double smoothing_alpha_;

    bool enable_player_detection_;
    double max_select_distance_m_;
    double selection_cooldown_sec_;
    double selection_hold_sec_;
    double min_hand_area_;
    double max_hand_area_;
    bool require_hand_release_;
    bool exclude_skin_from_block_masks_ = true;
    int skin_mask_open_iterations_ = 1;
    int skin_mask_close_iterations_ = 1;
    ros::Time last_selection_time_;
    int last_selected_block_id_;
    int stable_candidate_id_ = -1;
    ros::Time stable_candidate_since_;
    bool selection_armed_ = true;

    std::vector<ColorConfig> color_configs_;

    cv::Mat latest_depth_;
    std::string latest_depth_encoding_;
    ros::Time latest_depth_stamp_;
    bool has_depth_;

    image_geometry::PinholeCameraModel camera_model_;
    bool has_camera_model_;

    std::map<int, geometry_msgs::Point> filtered_positions_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

}  // namespace memory_game_vision

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");
    memory_game_vision::VisionNode node;
    ros::spin();
    return 0;
}
