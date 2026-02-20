/**
 * @file vision_node.cpp
 * @brief Vision node for detecting blocks and player interactions
 * 
 * Subscribes to:
 *   - /camera/color/image_raw (optional - can run in test mode)
 *   - /camera/depth/image_raw (optional - can run in test mode)
 * 
 * Publishes:
 *   - /detected_blocks (BlockArray)
 *   - /player_selection (PlayerSelection)
 *   - /visualization_marker (Marker) - for RViz visualization
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <memory_game/BlockArray.h>
#include <memory_game/Block.h>
#include <memory_game/PlayerSelection.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/ColorRGBA.h>

class VisionNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber color_sub_;
    ros::Subscriber depth_sub_;
    ros::Publisher blocks_pub_;
    ros::Publisher selection_pub_;
    ros::Publisher marker_pub_;
    
    bool test_mode_;  // If true, publish mock blocks without cameras
    ros::Timer test_timer_;
    
    // Mock block positions (for testing without cameras)
    std::vector<geometry_msgs::Point> mock_positions_;
    std::vector<std::string> mock_colors_;
    
public:
    VisionNode() {
        // Check if we should run in test mode (no cameras)
        nh_.param("test_mode", test_mode_, true);
        
        // Publishers
        blocks_pub_ = nh_.advertise<memory_game::BlockArray>("/detected_blocks", 10);
        selection_pub_ = nh_.advertise<memory_game::PlayerSelection>("/player_selection", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
        
        // Initialize mock block positions (4 blocks in a square pattern)
        initializeMockBlocks();
        
        if (test_mode_) {
            ROS_INFO("Vision node running in TEST MODE - publishing mock blocks");
            // Publish blocks periodically
            test_timer_ = nh_.createTimer(ros::Duration(1.0), &VisionNode::publishMockBlocks, this);
        } else {
            // Subscribers for real camera data
            color_sub_ = nh_.subscribe("/camera/color/image_raw", 10, 
                                       &VisionNode::colorCallback, this);
            depth_sub_ = nh_.subscribe("/camera/depth/image_raw", 10, 
                                        &VisionNode::depthCallback, this);
            ROS_INFO("Vision node initialized - waiting for camera data");
        }
    }
    
    void initializeMockBlocks() {
        // 4 blocks arranged in a square: red, green, blue, yellow
        mock_colors_ = {"red", "green", "blue", "yellow"};
        
        geometry_msgs::Point p;
        // Block 0: red (front-left)
        p.x = 0.4; p.y = 0.2; p.z = 0.05;
        mock_positions_.push_back(p);
        
        // Block 1: green (front-right)
        p.x = 0.4; p.y = -0.2; p.z = 0.05;
        mock_positions_.push_back(p);
        
        // Block 2: blue (back-left)
        p.x = 0.5; p.y = 0.2; p.z = 0.05;
        mock_positions_.push_back(p);
        
        // Block 3: yellow (back-right)
        p.x = 0.5; p.y = -0.2; p.z = 0.05;
        mock_positions_.push_back(p);
    }
    
    void publishMockBlocks(const ros::TimerEvent& event) {
        memory_game::BlockArray block_array;
        block_array.header.stamp = ros::Time::now();
        block_array.header.frame_id = "panda_link0";  // Robot base frame
        
        visualization_msgs::MarkerArray marker_array;
        
        for (int i = 0; i < 4; i++) {
            // Create Block message
            memory_game::Block block;
            block.header.stamp = ros::Time::now();
            block.header.frame_id = "panda_link0";
            block.id = i;
            block.color = mock_colors_[i];
            block.position = mock_positions_[i];
            block.orientation.w = 1.0;
            block.confidence = 0.9;
            block.is_selected = false;
            
            block_array.blocks.push_back(block);
            
            // Create visualization marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "panda_link0";
            marker.header.stamp = ros::Time::now();
            marker.ns = "blocks";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position = mock_positions_[i];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;  // 5cm cube
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            
            // Set color based on block color
            std_msgs::ColorRGBA color;
            if (mock_colors_[i] == "red") {
                color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
            } else if (mock_colors_[i] == "green") {
                color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 1.0;
            } else if (mock_colors_[i] == "blue") {
                color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = 1.0;
            } else if (mock_colors_[i] == "yellow") {
                color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 1.0;
            }
            marker.color = color;
            
            marker_array.markers.push_back(marker);
        }
        
        // Publish blocks and markers
        blocks_pub_.publish(block_array);
        marker_pub_.publish(marker_array);
        
        ROS_DEBUG("Published %zu mock blocks", block_array.blocks.size());
    }
    
    void colorCallback(const sensor_msgs::ImageConstPtr& msg) {
        // TODO: Implement block detection from RGB image
        ROS_DEBUG("Received color image");
    }
    
    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        // TODO: Implement depth processing
        ROS_DEBUG("Received depth image");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");
    
    VisionNode vision_node;
    
    ros::spin();
    
    return 0;
}
