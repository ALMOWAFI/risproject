/**
 * @file motion_node.cpp
 * @brief Motion control node for Panda robot
 * 
 * Subscribes to:
 *   - /target_block (Block) - block to point to
 * 
 * Publishes:
 *   - /motion_status (String) - motion status
 */

#include <ros/ros.h>
#include <memory_game/Block.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

class MotionNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Publisher status_pub_;
    ros::Publisher marker_pub_;  // Visualize target point in RViz
    
    geometry_msgs::Pose home_pose_;
    bool at_home_;
    
public:
    MotionNode() : at_home_(true) {
        // Subscribers
        target_sub_ = nh_.subscribe("/target_block", 10, 
                                    &MotionNode::targetCallback, this);
        
        // Publishers
        status_pub_ = nh_.advertise<std_msgs::String>("/motion_status", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/target_marker", 10);
        
        // Initialize home pose
        initializeHomePose();
        
        ROS_INFO("Motion node initialized");
    }
    
    void initializeHomePose() {
        // Safe home position for Panda
        home_pose_.position.x = 0.3;
        home_pose_.position.y = 0.0;
        home_pose_.position.z = 0.4;
        home_pose_.orientation.w = 1.0;
        
        ROS_INFO("Home pose initialized");
    }
    
    void targetCallback(const memory_game::Block::ConstPtr& msg) {
        ROS_INFO("Received target block: %d at position (%.2f, %.2f, %.2f)", 
                 msg->id, msg->position.x, msg->position.y, msg->position.z);
        
        publishStatus("MOVING");
        
        // Publish marker for RViz visualization
        publishTargetMarker(*msg);
        
        moveToBlock(*msg);
    }
    
    void publishTargetMarker(const memory_game::Block& block) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target";
        marker.id = 100;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Arrow from home to target
        geometry_msgs::Point start, end;
        start.x = home_pose_.position.x;
        start.y = home_pose_.position.y;
        start.z = home_pose_.position.z;
        
        end.x = block.position.x;
        end.y = block.position.y;
        end.z = block.position.z + 0.1;  // Point above block
        
        marker.points.push_back(start);
        marker.points.push_back(end);
        
        marker.scale.x = 0.02;  // Shaft diameter
        marker.scale.y = 0.03;  // Head diameter
        marker.scale.z = 0.0;
        
        std_msgs::ColorRGBA color;
        color.r = 1.0; color.g = 0.0; color.b = 1.0; color.a = 1.0;  // Magenta
        marker.color = color;
        
        marker.lifetime = ros::Duration(5.0);
        marker_pub_.publish(marker);
    }
    
    void moveToBlock(const memory_game::Block& block) {
        geometry_msgs::Pose target_pose;
        target_pose.position = block.position;
        
        // Offset for pointing (approach from above)
        target_pose.position.z += 0.1; // 10cm above block
        
        // Orientation: point downward
        target_pose.orientation.x = 0.707; // 90 degrees around x-axis
        target_pose.orientation.w = 0.707;
        
        ROS_INFO("Moving to block %d (%s)", block.id, block.color.c_str());
        
        // TODO: Execute motion using Franka/MoveIt
        // For now, simulate with delay
        ros::Duration(2.0).sleep();
        
        ROS_INFO("Reached block %d", block.id);
        publishStatus("DONE");
        
        // Return to home after pointing
        returnToHome();
    }
    
    void returnToHome() {
        ROS_INFO("Returning to home position");
        publishStatus("MOVING");
        
        // TODO: Execute motion to home pose
        // For now, simulate with delay
        ros::Duration(1.5).sleep();
        
        at_home_ = true;
        publishStatus("DONE");
        ROS_INFO("At home position");
    }
    
    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_node");
    
    MotionNode motion_node;
    
    ros::spin();
    
    return 0;
}
