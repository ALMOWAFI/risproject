/**
 * @file game_node.cpp
 * @brief Game logic node - manages sequences, scoring, and game state
 * 
 * Subscribes to:
 *   - /detected_blocks (BlockArray)
 *   - /player_selection (PlayerSelection)
 * 
 * Publishes:
 *   - /target_block (Block) - which block robot should point to
 *   - /game_state (String) - current game state
 *   - /score (Int32) - current score
 */
#include <ros/ros.h>
#include <memory_game/BlockArray.h>
#include <memory_game/PlayerSelection.h>
#include <memory_game/Block.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <random>
#include <map>
#include <vector>
#include <random>
#include <map>

enum GameState{
    IDLE,
    SHOWING_SEQUENCE,
    WAITING_PLAYER,
    CHECKING_INPUT,
    GAME_OVER
};

class GameNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber blocks_sub_;
    ros::Subscriber selection_sub_;
    ros::Publisher target_pub_;
    ros::Publisher state_pub_;
    ros::Publisher score_pub_;
    ros::Timer next_round_timer;

    GameState current_state_;
    std::vector<int> available_block_ids_;
    std::vector<int> sequence_;
    std::vector<int> player_input_;
    int current_sequence_index_;
    int current_player_index_;
    int score_;
    int level_;
    
    std::random_device rd_;
    std::mt19937 gen_;
    
    // Store detected blocks (for getting positions)
    std::map<int, memory_game::Block> known_blocks_;
    ros::Timer sequence_timer_;
    bool waiting_for_motion_complete_;
    
public:
    GameNode() : gen_(rd_()), current_state_(IDLE), score_(0), level_(1), 
                 waiting_for_motion_complete_(false) {
        // Subscribers
        blocks_sub_ = nh_.subscribe("/detected_blocks", 10, 
                                     &GameNode::blocksCallback, this);
        selection_sub_ = nh_.subscribe("/player_selection", 10, 
                                        &GameNode::selectionCallback, this);
        
        // Publishers
        target_pub_ = nh_.advertise<memory_game::Block>("/target_block", 10);
        state_pub_ = nh_.advertise<std_msgs::String>("/game_state", 10);
        score_pub_ = nh_.advertise<std_msgs::Int32>("/score", 10);
        
        ROS_INFO("Game node initialized");
    }
    
    void blocksCallback(const memory_game::BlockArray::ConstPtr& msg) {
        // Store detected blocks for reference
        for (const auto& block : msg->blocks) {
            known_blocks_[block.id] = block;
        }
        ROS_DEBUG("Received %zu detected blocks", msg->blocks.size());
    }
    
    void selectionCallback(const memory_game::PlayerSelection::ConstPtr& msg) {
        if (current_state_ == WAITING_PLAYER) {
            player_input_.push_back(msg->block_id);
            current_player_index_++;
            
            ROS_INFO("Player selected block %d (%s)", msg->block_id, msg->color.c_str());
            
            // Check if sequence is complete
            if (current_player_index_ >= sequence_.size()) {
                checkSequence();
            }
        }
    }
    
    void generateSequence() {
        sequence_.clear();
        std::uniform_int_distribution<> dis(0, 3); // 4 blocks (0-3)
        
        // Generate sequence based on level
        int sequence_length = level_ + 2; // Start with 3, increase by 1 each level
        for (int i = 0; i < sequence_length; i++) {
            sequence_.push_back(dis(gen_));
        }
        
        current_sequence_index_ = 0;
        current_player_index_ = 0;
        player_input_.clear();
        
        ROS_INFO("Generated sequence of length %d", sequence_length);
        ROS_INFO("Sequence: ");
        for (size_t i = 0; i < sequence_.size(); i++) {
            ROS_INFO("  [%zu] Block %d", i, sequence_[i]);
        }
        
        startShowingSequence();
    }
    
    void startShowingSequence() {
        current_state_ = SHOWING_SEQUENCE;
        publishState("SHOWING_SEQUENCE");
        waiting_for_motion_complete_ = false;
        showNextBlock();
    }
    
    void showNextBlock() {
        if (current_sequence_index_ < sequence_.size()) {
            int block_id = sequence_[current_sequence_index_];
            
            // Check if we have this block's position
            if (known_blocks_.find(block_id) != known_blocks_.end()) {
                memory_game::Block target = known_blocks_[block_id];
                target.id = block_id;
                target_pub_.publish(target);
                
                ROS_INFO("Showing block %d (%s) (%d/%zu)", 
                         block_id, target.color.c_str(), 
                         current_sequence_index_ + 1, sequence_.size());
            } else {
                // Block not detected yet, create minimal block message
                memory_game::Block target;
                target.header.stamp = ros::Time::now();
                target.header.frame_id = "panda_link0";
                target.id = block_id;
                target.color = "unknown";
                target.position.x = 0.4;
                target.position.y = 0.0;
                target.position.z = 0.05;
                target_pub_.publish(target);
                
                ROS_WARN("Block %d not detected, using default position", block_id);
            }
            
            current_sequence_index_++;
            
            // Wait before showing next block (simulate motion time)
            if (current_sequence_index_ < sequence_.size()) {
                sequence_timer_ = nh_.createTimer(ros::Duration(3.0), 
                                                  &GameNode::showNextBlockTimer, this, true);
            } else {
                // Sequence complete, wait for player
                sequence_timer_ = nh_.createTimer(ros::Duration(3.0), 
                                                  &GameNode::sequenceComplete, this, true);
            }
        }
    }
    
    void showNextBlockTimer(const ros::TimerEvent& event) {
        showNextBlock();
    }
    
    void sequenceComplete(const ros::TimerEvent& event) {
        current_state_ = WAITING_PLAYER;
        publishState("WAITING_PLAYER");
        ROS_INFO("Sequence complete. Waiting for player input...");
    }
    
    void checkSequence() {
        current_state_ = CHECKING_INPUT;
        publishState("CHECKING_INPUT");
        
        bool correct = true;
        for (size_t i = 0; i < sequence_.size(); i++) {
            if (i >= player_input_.size() || sequence_[i] != player_input_[i]) {
                correct = false;
                break;
            }
        }
        
        if (correct) {
            score_ += 10 * level_;
            level_++;
            ROS_INFO("✓ Correct! Score: %d, Level: %d", score_, level_);
            publishScore();
            
            // Start next round
            ros::Duration(2.0).sleep();
            generateSequence();
        } else {
            ROS_INFO("✗ Incorrect sequence. Game over!");
            ROS_INFO("Expected: ");
            for (size_t i = 0; i < sequence_.size(); i++) {
                ROS_INFO("  [%zu] Block %d", i, sequence_[i]);
            }
            ROS_INFO("Got: ");
            for (size_t i = 0; i < player_input_.size(); i++) {
                ROS_INFO("  [%zu] Block %d", i, player_input_[i]);
            }
            current_state_ = GAME_OVER;
            publishState("GAME_OVER");
        }
    }
    
    void publishState(const std::string& state) {
        std_msgs::String msg;
        msg.data = state;
        state_pub_.publish(msg);
        ROS_INFO("Game state: %s", state.c_str());
    }
    
    void publishScore() {
        std_msgs::Int32 msg;
        msg.data = score_;
        score_pub_.publish(msg);
    }
    
    void startGame() {
        ROS_INFO("Starting new game");
        score_ = 0;
        level_ = 1;
        // Wait a bit for vision node to publish blocks
        ros::Duration(2.0).sleep();
        generateSequence();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "game_node");
    
    GameNode game_node;
    
    // Start game after a brief delay
    ros::Duration(1.0).sleep();
    game_node.startGame();
    
    ros::spin();
    
    return 0;
}
