

#include <ros/ros.h>
#include <memory_game/BlockArray.h>
#include <memory_game/PlayerSelection.h>
#include <memory_game/Block.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>

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
    ros::Timer target_sub_wait_timer_;

    GameState current_state_;

    std::vector<int> sequence_;
    std::vector<int> player_input_;
    std::map<int, memory_game::Block> known_blocks_;

    int score_;
    int level_;
    int show_index_;
    int player_index_;

    bool waiting_for_motion_;
    bool motion_started_for_sequence_;
    bool have_motion_state_;
    std::string last_motion_state_;

    std::mt19937 gen_;

    // Parameters
    int num_blocks_;
    int base_length_;
    int length_per_level_;
    double show_hold_sec_;
    double between_show_sec_;
    double round_pause_sec_;
    double player_timeout_sec_;
    double start_delay_sec_;
    bool no_immediate_repeat_;
    bool require_detected_blocks_;
    double blocks_wait_sec_;
    // When using batch-publish mode, publishing /target_block before motion starts can drop messages.
    // This makes the "show sequence" incomplete with no obvious error, so we wait for a subscriber.
    double target_subscriber_poll_sec_;
    double target_subscriber_timeout_sec_;

    geometry_msgs::Point default_target_;
    std::string target_frame_;
    // Guard against publishing the same sequence multiple times while we wait for subscribers.
    bool sequence_sent_to_motion_;
    ros::Time target_sub_wait_start_;

public:
    GameNode()
        : pnh_("~"),
          current_state_(GameState::IDLE),
          score_(0),
          level_(1),
          show_index_(0),
          player_index_(0),
          waiting_for_motion_(false),
          motion_started_for_sequence_(false),
          have_motion_state_(false),
          gen_(std::random_device{}()),
          sequence_sent_to_motion_(false) {
        loadParams();

        blocks_sub_ = nh_.subscribe("/detected_blocks", 10, &GameNode::blocksCallback, this);
        selection_sub_ = nh_.subscribe("/player_selection", 10, &GameNode::selectionCallback, this);
        motion_sub_ = nh_.subscribe("/motion_status", 10, &GameNode::motionCallback, this);

        target_pub_ = nh_.advertise<memory_game::Block>("/target_block", 10);
        state_pub_ = nh_.advertise<std_msgs::String>("/game_state", 10, true);
        score_pub_ = nh_.advertise<std_msgs::Int32>("/score", 10, true);

        publishState("IDLE");
        publishScore();

        start_timer_ = nh_.createTimer(
            ros::Duration(start_delay_sec_),
            &GameNode::startTimerCallback,
            this,
            true);

        ROS_INFO("Game node initialized");
    }

private:
    void loadParams() {
        pnh_.param("num_blocks", num_blocks_, 4);
        pnh_.param("base_length", base_length_, 3);
        pnh_.param("length_per_level", length_per_level_, 1);
        pnh_.param("show_hold_sec", show_hold_sec_, 1.0);
        pnh_.param("between_show_sec", between_show_sec_, 0.3);
        pnh_.param("round_pause_sec", round_pause_sec_, 1.0);
        // Support both names: player_timeout_sec (new) and player_timeout (legacy/config file).
        if (!pnh_.getParam("player_timeout_sec", player_timeout_sec_)) {
            if (!pnh_.getParam("player_timeout", player_timeout_sec_)) {
                player_timeout_sec_ = 12.0;
            }
        }
        pnh_.param("start_delay_sec", start_delay_sec_, 0.8);
        pnh_.param("no_immediate_repeat", no_immediate_repeat_, true);
        pnh_.param("target_frame", target_frame_, std::string("panda_link0"));
        pnh_.param("require_detected_blocks", require_detected_blocks_, false);
        pnh_.param("blocks_wait_sec", blocks_wait_sec_, 0.5);
        pnh_.param("target_subscriber_poll_sec", target_subscriber_poll_sec_, 0.2);
        pnh_.param("target_subscriber_timeout_sec", target_subscriber_timeout_sec_, 5.0);

        pnh_.param("default_x", default_target_.x, 0.40);
        pnh_.param("default_y", default_target_.y, 0.00);
        pnh_.param("default_z", default_target_.z, 0.05);

        int seed = -1;
        pnh_.param("seed", seed, -1);
        if (seed >= 0) {
            gen_.seed(static_cast<unsigned int>(seed));
        }

        num_blocks_ = std::max(1, num_blocks_);
        base_length_ = std::max(1, base_length_);
        length_per_level_ = std::max(0, length_per_level_);
        show_hold_sec_ = std::max(0.1, show_hold_sec_);
        // <= 0 means disabled timeout (useful while debugging vision/player input).
        if (player_timeout_sec_ < 0.0) {
            player_timeout_sec_ = 0.0;
        }
        blocks_wait_sec_ = std::max(0.05, blocks_wait_sec_);
        target_subscriber_poll_sec_ = std::max(0.05, target_subscriber_poll_sec_);
        target_subscriber_timeout_sec_ = std::max(0.0, target_subscriber_timeout_sec_);
    }

    bool haveAllBlocks() const {
        if (known_blocks_.size() < static_cast<size_t>(num_blocks_)) {
            return false;
        }
        for (int id = 0; id < num_blocks_; ++id) {
            if (known_blocks_.find(id) == known_blocks_.end()) {
                return false;
            }
        }
        return true;
    }

    void blocksCallback(const memory_game::BlockArray::ConstPtr& msg) {
        for (const auto& block : msg->blocks) {
            known_blocks_[block.id] = block;
        }
    }

    void motionCallback(const std_msgs::String::ConstPtr& msg) {
        last_motion_state_ = msg->data;
        have_motion_state_ = true;

        if (current_state_ == GameState::SHOWING_SEQUENCE && waiting_for_motion_) {
            if (last_motion_state_ == "MOVING_TO_TARGET" ||
                last_motion_state_ == "AT_TARGET" ||
                last_motion_state_ == "RETURNING_HOME") {
                motion_started_for_sequence_ = true;
            }

            // Sequence is done once motion returns to IDLE after executing queued targets.
            if (last_motion_state_ == "IDLE" && motion_started_for_sequence_) {
                waiting_for_motion_ = false;
                motion_started_for_sequence_ = false;
                current_state_ = GameState::WAITING_PLAYER;
                publishState("WAITING_PLAYER");
                ROS_INFO("Sequence complete. Waiting for player input...");
                resetPlayerTimeout();
            }
        }
    }

    void selectionCallback(const memory_game::PlayerSelection::ConstPtr& msg) {
        if (current_state_ != GameState::WAITING_PLAYER) {
            return;
        }

        player_input_.push_back(msg->block_id);
        player_index_++;

        ROS_INFO("Player selected block %d (%s)", msg->block_id, msg->color.c_str());

        resetPlayerTimeout();

        if (player_index_ >= static_cast<int>(sequence_.size())) {
            current_state_ = GameState::CHECKING_INPUT;
            publishState("CHECKING_INPUT");
            checkSequence();
        }
    }

    void startTimerCallback(const ros::TimerEvent&) {
        startGame();
    }

    void showHoldCallback(const ros::TimerEvent&) {
        if (between_show_sec_ > 1e-6) {
            show_timer_ = nh_.createTimer(
                ros::Duration(between_show_sec_),
                &GameNode::showNextCallback,
                this,
                true);
        } else {
            showNextBlock();
        }
    }

    void showNextCallback(const ros::TimerEvent&) {
        showNextBlock();
    }

    void showFailsafeCallback(const ros::TimerEvent&) {
        if (current_state_ == GameState::SHOWING_SEQUENCE && waiting_for_motion_) {
            ROS_WARN("Motion sequence completion timeout; switching to player input");
            waiting_for_motion_ = false;
            motion_started_for_sequence_ = false;
            current_state_ = GameState::WAITING_PLAYER;
            publishState("WAITING_PLAYER");
            resetPlayerTimeout();
        }
    }

    void roundPauseCallback(const ros::TimerEvent&) {
        generateAndStartSequence();
    }

    void targetSubWaitCallback(const ros::TimerEvent&) {
        // Retry sending the sequence once motion is subscribed to /target_block.
        sendSequenceBatchToMotion();
    }

    void playerTimeoutCallback(const ros::TimerEvent&) {
        if (current_state_ != GameState::WAITING_PLAYER) {
            return;
        }

        ROS_WARN("Player timeout. Game over.");
        gameOver();
    }

    void startGame() {
        // On real hardware, don't start showing a sequence until we have block poses from vision.
        // This avoids motion being driven by the default/fallback target.
        if (require_detected_blocks_ && !haveAllBlocks()) {
            publishState("WAITING_FOR_BLOCKS");
            ROS_WARN_THROTTLE(2.0, "Waiting for /detected_blocks (%zu/%d blocks cached)",
                              known_blocks_.size(), num_blocks_);
            start_timer_ = nh_.createTimer(
                ros::Duration(blocks_wait_sec_),
                &GameNode::startTimerCallback,
                this,
                true);
            return;
        }

        ROS_INFO("Starting new game");
        score_ = 0;
        level_ = 1;
        publishScore();
        generateAndStartSequence();
    }

    void generateAndStartSequence() {
        sequence_.clear();
        player_input_.clear();
        show_index_ = 0;
        player_index_ = 0;
        waiting_for_motion_ = false;
        motion_started_for_sequence_ = false;
        sequence_sent_to_motion_ = false;
        target_sub_wait_start_ = ros::Time(0);
        target_sub_wait_timer_.stop();

        int sequence_length = base_length_ + length_per_level_ * (level_ - 1);
        std::uniform_int_distribution<int> dis(0, num_blocks_ - 1);

        int prev = -1;
        for (int i = 0; i < sequence_length; i++) {
            int value = dis(gen_);
            if (no_immediate_repeat_ && num_blocks_ > 1) {
                while (value == prev) {
                    value = dis(gen_);
                }
            }
            sequence_.push_back(value);
            prev = value;
        }

        current_state_ = GameState::SHOWING_SEQUENCE;
        publishState("SHOWING_SEQUENCE");

        ROS_INFO("Generated sequence of length %d", sequence_length);
        for (size_t i = 0; i < sequence_.size(); i++) {
            ROS_INFO("  [%zu] Block %d", i, sequence_[i]);
        }

        sendSequenceBatchToMotion();
    }

    void sendSequenceBatchToMotion() {
        if (sequence_sent_to_motion_) {
            return;
        }

        if (target_pub_.getNumSubscribers() == 0) {
            // Ensure motion is actually listening before we send the sequence.
            // In ROS topics, early publishes can be dropped if the subscriber connects later.
            const ros::Time now = ros::Time::now();
            if (target_sub_wait_start_.isZero()) {
                target_sub_wait_start_ = now;
            }

            const double waited_sec = (now - target_sub_wait_start_).toSec();
            if (waited_sec > target_subscriber_timeout_sec_) {
                ROS_WARN("No /target_block subscribers after %.1fs. Proceeding to player input.", waited_sec);
                sequence_sent_to_motion_ = true;
                waiting_for_motion_ = false;
                motion_started_for_sequence_ = false;
                current_state_ = GameState::WAITING_PLAYER;
                publishState("WAITING_PLAYER");
                resetPlayerTimeout();
                return;
            }

            ROS_WARN_THROTTLE(2.0, "Waiting for /target_block subscriber (%.1fs/%.1fs)",
                              waited_sec, target_subscriber_timeout_sec_);
            publishState("WAITING_FOR_MOTION");
            target_sub_wait_timer_ = nh_.createTimer(
                ros::Duration(target_subscriber_poll_sec_),
                &GameNode::targetSubWaitCallback,
                this,
                true);
            return;
        }

        waiting_for_motion_ = true;
        motion_started_for_sequence_ = false;
        sequence_sent_to_motion_ = true;

        for (size_t i = 0; i < sequence_.size(); ++i) {
            const int block_id = sequence_[i];
            memory_game::Block target = makeTargetBlock(block_id);
            target_pub_.publish(target);
            ROS_INFO("Queued block %d (%zu/%zu)", block_id, i + 1, sequence_.size());
        }

        if (!have_motion_state_) {
            ROS_WARN("No motion status detected. Proceeding to player input.");
            waiting_for_motion_ = false;
            current_state_ = GameState::WAITING_PLAYER;
            publishState("WAITING_PLAYER");
            resetPlayerTimeout();
            return;
        }

        const double per_target_budget_sec = std::max(2.0, show_hold_sec_ + between_show_sec_ + 1.5);
        const double timeout_sec = std::max(5.0, per_target_budget_sec * sequence_.size() + 2.0);
        show_failsafe_timer_ = nh_.createTimer(
            ros::Duration(timeout_sec),
            &GameNode::showFailsafeCallback,
            this,
            true);
    }

    void showNextBlock() {
        if (current_state_ != GameState::SHOWING_SEQUENCE) {
            return;
        }

        if (show_index_ >= static_cast<int>(sequence_.size())) {
            current_state_ = GameState::WAITING_PLAYER;
            publishState("WAITING_PLAYER");
            ROS_INFO("Sequence complete. Waiting for player input...");
            resetPlayerTimeout();
            return;
        }

        int block_id = sequence_[show_index_];
        memory_game::Block target = makeTargetBlock(block_id);
        target_pub_.publish(target);

        ROS_INFO("Showing block %d (%d/%zu)",
                 block_id,
                 show_index_ + 1,
                 sequence_.size());

        show_index_++;
        waiting_for_motion_ = true;

        if (!have_motion_state_) {
            waiting_for_motion_ = false;
            show_timer_ = nh_.createTimer(
                ros::Duration(show_hold_sec_),
                &GameNode::showHoldCallback,
                this,
                true);
        } else {
            show_failsafe_timer_ = nh_.createTimer(
                ros::Duration(std::max(3.0, show_hold_sec_ + between_show_sec_ + 2.0)),
                &GameNode::showFailsafeCallback,
                this,
                true);
        }
    }

    void checkSequence() {
        bool correct = true;

        if (player_input_.size() != sequence_.size()) {
            correct = false;
        }

        for (size_t i = 0; correct && i < sequence_.size(); i++) {
            if (sequence_[i] != player_input_[i]) {
                correct = false;
            }
        }

        if (correct) {
            score_ += 10 * level_;
            level_++;
            publishScore();

            ROS_INFO("Correct! Score: %d, Level: %d", score_, level_);

            current_state_ = GameState::IDLE;
            publishState("ROUND_PAUSE");

            round_timer_ = nh_.createTimer(
                ros::Duration(round_pause_sec_),
                &GameNode::roundPauseCallback,
                this,
                true);
        } else {
            ROS_INFO("Incorrect sequence. Game over!");
            ROS_INFO("Expected:");
            for (size_t i = 0; i < sequence_.size(); i++) {
                ROS_INFO("  [%zu] Block %d", i, sequence_[i]);
            }
            ROS_INFO("Got:");
            for (size_t i = 0; i < player_input_.size(); i++) {
                ROS_INFO("  [%zu] Block %d", i, player_input_[i]);
            }

            gameOver();
        }
    }

    void gameOver() {
        current_state_ = GameState::GAME_OVER;
        publishState("GAME_OVER");

        player_timeout_timer_.stop();
        show_timer_.stop();
        show_failsafe_timer_.stop();
        round_timer_.stop();
        target_sub_wait_timer_.stop();
    }

    void resetPlayerTimeout() {
        player_timeout_timer_.stop();
        if (player_timeout_sec_ <= 0.0) {
            ROS_INFO_THROTTLE(2.0, "Player timeout disabled (player_timeout_sec <= 0)");
            return;
        }
        player_timeout_timer_ = nh_.createTimer(
            ros::Duration(player_timeout_sec_),
            &GameNode::playerTimeoutCallback,
            this,
            true);
    }

    memory_game::Block makeTargetBlock(int block_id) const {
        auto it = known_blocks_.find(block_id);
        if (it != known_blocks_.end()) {
            memory_game::Block target = it->second;
            target.header.stamp = ros::Time::now();
            target.id = block_id;
            return target;
        }

        memory_game::Block target;
        target.header.stamp = ros::Time::now();
        target.header.frame_id = target_frame_;
        target.id = block_id;
        target.color = "unknown";
        target.position = default_target_;
        target.orientation.w = 1.0;
        target.confidence = 0.0;
        target.is_selected = false;
        return target;
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "game_node");
    GameNode game_node;
    ros::spin();
    return 0;
}
