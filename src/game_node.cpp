/**
 * @file game_node.cpp
 * @brief Game logic node - manages sequences, scoring, and game state (final).
 *
 * Subscribes to:
 *   - /detected_blocks (memory_game/BlockArray)
 *   - /player_selection (memory_game/PlayerSelection)
 *   - /motion_status (std_msgs/String)
 *
 * Publishes:
 *   - /target_block (memory_game/Block)
 *   - /game_state (std_msgs/String)
 *   - /score (std_msgs/Int32)
 *
 * Key behavior:
 *   - Non-blocking timers (no sleep)
 *   - Sequence display synchronized with motion_node (wait for AT_TARGET)
 *   - Player input timeout
 *   - Param-driven game tuning
 */

#include <ros/ros.h>
#include <memory_game/BlockArray.h>
#include <memory_game/PlayerSelection.h>
#include <memory_game/Block.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <vector>
#include <map>
#include <random>
#include <string>
#include <algorithm>

enum class GameState {
    IDLE,
    SHOWING_SEQUENCE,
    WAITING_PLAYER,
    CHECKING_INPUT,
    GAME_OVER
};

class GameNode {
public:
    GameNode()
        : nh_(),
          pnh_("~"),
          gen_(std::random_device{}()),
          state_(GameState::IDLE),
          score_(0),
          level_(1),
          show_index_(0),
          player_index_(0),
          waiting_motion_(false),
          have_motion_state_(false)
    {
        loadParams();

        blocks_sub_ = nh_.subscribe("/detected_blocks", 10, &GameNode::blocksCallback, this);
        selection_sub_ = nh_.subscribe("/player_selection", 10, &GameNode::selectionCallback, this);
        motion_sub_ = nh_.subscribe("/motion_status", 10, &GameNode::motionCallback, this);

        target_pub_ = nh_.advertise<memory_game::Block>("/target_block", 10);
        state_pub_  = nh_.advertise<std_msgs::String>("/game_state", 10, true);
        score_pub_  = nh_.advertise<std_msgs::Int32>("/score", 10, true);

        publishState("IDLE");
        publishScore();

        // Start shortly after launch (no sleep blocking)
        start_timer_ = nh_.createTimer(ros::Duration(start_delay_sec_), &GameNode::startTimerCb, this, true);

        ROS_INFO("game_node ready (final).");
    }

private:
    // ---------------- Params ----------------
    void loadParams() {
        pnh_.param("num_blocks", num_blocks_, 4);
        pnh_.param("base_length", base_length_, 3);
        pnh_.param("length_per_level", length_per_level_, 1);

        pnh_.param("show_hold_sec", show_hold_sec_, 1.0);         // how long to stay at each shown block
        pnh_.param("between_show_sec", between_show_sec_, 0.3);   // optional gap between blocks
        pnh_.param("round_pause_sec", round_pause_sec_, 1.0);     // pause between rounds
        pnh_.param("player_timeout_sec", player_timeout_sec_, 12.0);
        pnh_.param("start_delay_sec", start_delay_sec_, 0.8);

        pnh_.param("no_immediate_repeat", no_immediate_repeat_, true);

        int seed_param = -1;
        pnh_.param("seed", seed_param, -1);
        if (seed_param >= 0) {
            gen_.seed(static_cast<unsigned>(seed_param));
        }

        // default target pose if block not detected yet
        pnh_.param("default_x", default_target_.x, 0.40);
        pnh_.param("default_y", default_target_.y, 0.00);
        pnh_.param("default_z", default_target_.z, 0.05);

        pnh_.param("target_frame", target_frame_, std::string("panda_link0"));

        num_blocks_ = std::max(1, num_blocks_);
        base_length_ = std::max(1, base_length_);
        length_per_level_ = std::max(0, length_per_level_);
        show_hold_sec_ = std::max(0.1, show_hold_sec_);
        player_timeout_sec_ = std::max(1.0, player_timeout_sec_);
    }

    // ---------------- Callbacks ----------------
    void blocksCallback(const memory_game::BlockArray::ConstPtr& msg) {
        for (const auto& b : msg->blocks) {
            known_blocks_[b.id] = b;
        }
    }

    void motionCallback(const std_msgs::String::ConstPtr& msg) {
        last_motion_state_ = msg->data;
        have_motion_state_ = true;

        // If we are showing sequence and waiting for the robot to reach target, advance on AT_TARGET
        if (state_ == GameState::SHOWING_SEQUENCE && waiting_motion_) {
            if (last_motion_state_ == "AT_TARGET") {
                waiting_motion_ = false;
                // hold at target briefly, then proceed
                show_timer_ = nh_.createTimer(ros::Duration(show_hold_sec_), &GameNode::showHoldCb, this, true);
            }
        }
    }

    void selectionCallback(const memory_game::PlayerSelection::ConstPtr& msg) {
        if (state_ != GameState::WAITING_PLAYER) return;

        player_input_.push_back(msg->block_id);
        player_index_++;

        ROS_INFO("Player selected: id=%d color=%s (%d/%zu)",
                 msg->block_id, msg->color.c_str(), player_index_, sequence_.size());

        // Reset timeout on every valid click
        resetPlayerTimeout();

        if (player_index_ >= static_cast<int>(sequence_.size())) {
            // done
            state_ = GameState::CHECKING_INPUT;
            publishState("CHECKING_INPUT");
            checkSequence();
        }
    }

    // ---------------- Timers ----------------
    void startTimerCb(const ros::TimerEvent&) {
        startGame();
    }

    void showHoldCb(const ros::TimerEvent&) {
        // optional small gap, then show next
        if (between_show_sec_ > 1e-6) {
            show_timer_ = nh_.createTimer(ros::Duration(between_show_sec_), &GameNode::showNextCb, this, true);
        } else {
            showNextCb(ros::TimerEvent{});
        }
    }

    void showNextCb(const ros::TimerEvent&) {
        showNextBlock();
    }

    void roundPauseCb(const ros::TimerEvent&) {
        generateAndStartSequence();
    }

    void playerTimeoutCb(const ros::TimerEvent&) {
        if (state_ != GameState::WAITING_PLAYER) return;
        ROS_WARN("Player timeout after %.1f sec. Game over.", player_timeout_sec_);
        gameOver();
    }

    // ---------------- Game Flow ----------------
    void startGame() {
        score_ = 0;
        level_ = 1;
        publishScore();

        ROS_INFO("Starting new game.");
        generateAndStartSequence();
    }

    void generateAndStartSequence() {
        sequence_.clear();
        player_input_.clear();
        show_index_ = 0;
        player_index_ = 0;
        waiting_motion_ = false;

        const int seq_len = base_length_ + length_per_level_ * (level_ - 1);

        std::uniform_int_distribution<int> dis(0, num_blocks_ - 1);
        int prev = -1;
        for (int i = 0; i < seq_len; ++i) {
            int x = dis(gen_);
            if (no_immediate_repeat_ && num_blocks_ > 1) {
                while (x == prev) x = dis(gen_);
            }
            sequence_.push_back(x);
            prev = x;
        }

        ROS_INFO("Level %d | sequence length %d", level_, seq_len);
        state_ = GameState::SHOWING_SEQUENCE;
        publishState("SHOWING_SEQUENCE");

        // If no motion_node running, we still "work" by using a fixed timer fallback.
        // But since you have motion_node, we try to sync on /motion_status.
        showNextBlock();
    }

    void showNextBlock() {
        if (state_ != GameState::SHOWING_SEQUENCE) return;

        if (show_index_ >= static_cast<int>(sequence_.size())) {
            // done showing
            state_ = GameState::WAITING_PLAYER;
            publishState("WAITING_PLAYER");
            ROS_INFO("Sequence complete. Waiting for player input...");
            resetPlayerTimeout();
            return;
        }

        const int block_id = sequence_[show_index_];

        memory_game::Block target = makeTargetBlock(block_id);
        target_pub_.publish(target);

        ROS_INFO("Show [%d/%zu] -> block_id=%d color=%s",
                 show_index_ + 1, sequence_.size(), block_id, target.color.c_str());

        show_index_++;

        // Wait until motion reports AT_TARGET, otherwise fallback timer if we never receive motion_status.
        waiting_motion_ = true;

        if (!have_motion_state_) {
            // fallback: assume it reaches in ~ show_hold_sec_
            waiting_motion_ = false;
            show_timer_ = nh_.createTimer(ros::Duration(show_hold_sec_), &GameNode::showHoldCb, this, true);
        } else {
            // safety fallback in case motion_status is alive but never reaches
            show_failsafe_timer_ =
                nh_.createTimer(ros::Duration(show_hold_sec_ + 2.0), &GameNode::showFailSafeCb, this, true);
        }
    }

    void showFailSafeCb(const ros::TimerEvent&) {
        if (state_ == GameState::SHOWING_SEQUENCE && waiting_motion_) {
            ROS_WARN("No AT_TARGET received in time. Advancing sequence by failsafe timer.");
            waiting_motion_ = false;
            show_timer_ = nh_.createTimer(ros::Duration(0.01), &GameNode::showNextCb, this, true);
        }
    }

    void checkSequence() {
        bool correct = true;
        if (player_input_.size() != sequence_.size()) correct = false;

        for (size_t i = 0; correct && i < sequence_.size(); ++i) {
            if (sequence_[i] != player_input_[i]) {
                correct = false;
                break;
            }
        }

        if (correct) {
            score_ += 10 * level_;
            level_++;
            publishScore();

            ROS_INFO("✓ Correct! Score=%d Level=%d", score_, level_);

            // Next round after short pause (no sleep)
            state_ = GameState::IDLE;
            publishState("ROUND_PAUSE");
            round_timer_ = nh_.createTimer(ros::Duration(round_pause_sec_), &GameNode::roundPauseCb, this, true);
        } else {
            ROS_INFO("✗ Incorrect. Game over.");
            gameOver();
        }
    }

    void gameOver() {
        state_ = GameState::GAME_OVER;
        publishState("GAME_OVER");
        // stop timers
        player_timeout_timer_.stop();
        show_timer_.stop();
        show_failsafe_timer_.stop();
        round_timer_.stop();
    }

    // ---------------- Helpers ----------------
    void resetPlayerTimeout() {
        player_timeout_timer_.stop();
        player_timeout_timer_ =
            nh_.createTimer(ros::Duration(player_timeout_sec_), &GameNode::playerTimeoutCb, this, true);
    }

    memory_game::Block makeTargetBlock(int block_id) const {
        memory_game::Block t;

        // If detected from vision, use its pose (this is the correct pipeline)
        auto it = known_blocks_.find(block_id);
        if (it != known_blocks_.end()) {
            t = it->second;
            t.id = block_id;
            // keep frame_id from vision, but ensure sane header stamp
            t.header.stamp = ros::Time::now();
            return t;
        }

        // Otherwise create a minimal block message with default pose
        t.header.stamp = ros::Time::now();
        t.header.frame_id = target_frame_;
        t.id = block_id;
        t.color = "unknown";
        t.position = default_target_;
        t.orientation.w = 1.0;
        t.confidence = 0.0;
        t.is_selected = false;
        return t;
    }

    void publishState(const std::string& s) {
        std_msgs::String msg;
        msg.data = s;
        state_pub_.publish(msg);
        ROS_INFO("Game state: %s", s.c_str());
    }

    void publishScore() {
        std_msgs::Int32 msg;
        msg.data = score_;
        score_pub_.publish(msg);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber blocks_sub_;
    ros::Subscriber selection_sub_;
    ros::Subscriber motion_sub_;

    ros::Publisher target_pub_;
    ros::Publisher state_pub_;
    ros::Publisher score_pub_;

    ros::Timer start_timer_;
    ros::Timer show_timer_;
    ros::Timer show_failsafe_timer_;
    ros::Timer round_timer_;
    ros::Timer player_timeout_timer_;

    std::mt19937 gen_;

    GameState state_;

    int score_;
    int level_;

    std::vector<int> sequence_;
    std::vector<int> player_input_;

    int show_index_;
    int player_index_;

    bool waiting_motion_;
    bool have_motion_state_;
    std::string last_motion_state_;

    std::map<int, memory_game::Block> known_blocks_;

    // Params
    int num_blocks_;
    int base_length_;
    int length_per_level_;

    double show_hold_sec_;
    double between_show_sec_;
    double round_pause_sec_;
    double player_timeout_sec_;
    double start_delay_sec_;

    bool no_immediate_repeat_;

    geometry_msgs::Point default_target_;
    std::string target_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "game_node");
    GameNode node;
    ros::spin();
    return 0;
}
