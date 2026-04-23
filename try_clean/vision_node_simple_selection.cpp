#include <cmath>
#include <map>
#include <string>

#include <geometry_msgs/Point.h>
#include <memory_game/BlockArray.h>
#include <memory_game/PlayerSelection.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

struct SlotState {
    int block_id = -1;
    std::string color;
    geometry_msgs::Point position;
    ros::Time last_seen;
    ros::Time missing_since;
    bool selected_latched = false;
};

class SimpleSelectionNode {
public:
    SimpleSelectionNode() : pnh_("~") {
        pnh_.param("target_frame", target_frame_, std::string("panda_link0"));
        pnh_.param("selection_type", selection_type_, std::string("slot_disappeared"));
        pnh_.param("max_slot_distance_m", max_slot_distance_m_, 0.08);
        pnh_.param("missing_hold_sec", missing_hold_sec_, 1.0);
        pnh_.param("selection_cooldown_sec", selection_cooldown_sec_, 1.0);
        pnh_.param("freeze_slots_on_first_frame", freeze_slots_on_first_frame_, true);
        pnh_.param("only_waiting_player", only_waiting_player_, true);

        selection_pub_ = nh_.advertise<memory_game::PlayerSelection>("/player_selection", 10);
        blocks_sub_ = nh_.subscribe("/detected_blocks", 10, &SimpleSelectionNode::blocksCallback, this);
        game_state_sub_ = nh_.subscribe("/game_state", 10, &SimpleSelectionNode::gameStateCallback, this);

        ROS_INFO("vision_node_simple_selection ready");
    }

private:
    void gameStateCallback(const std_msgs::String::ConstPtr& msg) {
        if (!msg) {
            return;
        }

        const std::string prev = current_game_state_;
        current_game_state_ = msg->data;
        if (current_game_state_ == "WAITING_PLAYER" && prev != "WAITING_PLAYER") {
            slots_initialized_ = false;
            slots_.clear();
        }
    }

    void blocksCallback(const memory_game::BlockArray::ConstPtr& msg) {
        if (!msg) {
            return;
        }

        const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        if (only_waiting_player_ && current_game_state_ != "WAITING_PLAYER") {
            resetMissingState(now);
            return;
        }

        std::map<int, memory_game::Block> observed;
        for (const auto& block : msg->blocks) {
            observed[block.id] = block;
        }

        if (!slots_initialized_) {
            initializeSlots(observed, now);
            return;
        }

        for (auto& entry : slots_) {
            SlotState& slot = entry.second;
            auto observed_it = observed.find(slot.block_id);
            const bool slot_occupied =
                observed_it != observed.end() && matchesSlot(slot, observed_it->second);

            if (slot_occupied) {
                slot.last_seen = now;
                slot.missing_since = ros::Time(0);
                slot.selected_latched = false;
                continue;
            }

            if (slot.selected_latched) {
                continue;
            }

            if (slot.missing_since.isZero()) {
                slot.missing_since = now;
                continue;
            }

            const double held_missing = (now - slot.missing_since).toSec();
            const double cooldown = (now - last_selection_time_).toSec();
            if (held_missing >= missing_hold_sec_ && cooldown >= selection_cooldown_sec_) {
                publishSelection(slot, now);
                slot.selected_latched = true;
            }
        }
    }

    void initializeSlots(const std::map<int, memory_game::Block>& observed, const ros::Time& now) {
        if (observed.empty()) {
            return;
        }

        for (const auto& entry : observed) {
            SlotState slot;
            slot.block_id = entry.first;
            slot.color = entry.second.color;
            slot.position = entry.second.position;
            slot.last_seen = now;
            slots_[entry.first] = slot;
        }

        slots_initialized_ = freeze_slots_on_first_frame_;
        if (slots_initialized_) {
            ROS_INFO("Initialized %zu selection slots", slots_.size());
        }
    }

    void resetMissingState(const ros::Time& now) {
        for (auto& entry : slots_) {
            entry.second.last_seen = now;
            entry.second.missing_since = ros::Time(0);
            entry.second.selected_latched = false;
        }
    }

    bool matchesSlot(const SlotState& slot, const memory_game::Block& block) const {
        const double dx = slot.position.x - block.position.x;
        const double dy = slot.position.y - block.position.y;
        const double dz = slot.position.z - block.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) <= max_slot_distance_m_;
    }

    void publishSelection(const SlotState& slot, const ros::Time& stamp) {
        memory_game::PlayerSelection msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = target_frame_;
        msg.block_id = slot.block_id;
        msg.color = slot.color;
        msg.selection_type = selection_type_;
        msg.selection_time = stamp;
        msg.confidence = 1.0;
        selection_pub_.publish(msg);

        last_selection_time_ = stamp;
        ROS_INFO("Simple selection: block %d (%s)", slot.block_id, slot.color.c_str());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber blocks_sub_;
    ros::Subscriber game_state_sub_;
    ros::Publisher selection_pub_;

    std::string target_frame_;
    std::string selection_type_;
    std::string current_game_state_ = "IDLE";
    double max_slot_distance_m_ = 0.08;
    double missing_hold_sec_ = 1.0;
    double selection_cooldown_sec_ = 1.0;
    bool freeze_slots_on_first_frame_ = true;
    bool only_waiting_player_ = true;
    bool slots_initialized_ = false;

    ros::Time last_selection_time_;
    std::map<int, SlotState> slots_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node_simple_selection");
    SimpleSelectionNode node;
    ros::spin();
    return 0;
}
