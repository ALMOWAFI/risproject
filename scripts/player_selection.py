#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, Optional

import rospy
from std_msgs.msg import String

from memory_game.msg import BlockArray, PlayerSelection


@dataclass
class SlotState:
    block_id: int
    color: str
    x: float
    y: float
    z: float
    last_seen: rospy.Time
    missing_since: Optional[rospy.Time] = None
    selected_latched: bool = False


class PlayerSelectionNode:
    def __init__(self) -> None:
        self.target_frame = rospy.get_param("~target_frame", "panda_link0")
        self.selection_type = rospy.get_param("~selection_type", "slot_disappeared")
        self.max_slot_distance_m = float(rospy.get_param("~max_slot_distance_m", 0.08))
        self.missing_hold_sec = float(rospy.get_param("~missing_hold_sec", 1.0))
        self.selection_cooldown_sec = float(rospy.get_param("~selection_cooldown_sec", 1.0))
        self.freeze_slots_on_first_frame = bool(rospy.get_param("~freeze_slots_on_first_frame", True))
        self.only_waiting_player = bool(rospy.get_param("~only_waiting_player", True))

        self.current_game_state = "IDLE"
        self.previous_game_state = "IDLE"
        self.last_selection_time = rospy.Time(0)
        self.slots_initialized = False
        self.slots: Dict[int, SlotState] = {}

        self.selection_pub = rospy.Publisher("/player_selection", PlayerSelection, queue_size=10)
        rospy.Subscriber("/detected_blocks", BlockArray, self.blocks_callback, queue_size=10)
        rospy.Subscriber("/game_state", String, self.game_state_callback, queue_size=10)

        rospy.loginfo("player_selection.py ready")

    def game_state_callback(self, msg: String) -> None:
        self.previous_game_state = self.current_game_state
        self.current_game_state = msg.data
        if self.current_game_state == "WAITING_PLAYER" and self.previous_game_state != "WAITING_PLAYER":
            self.slots_initialized = False
            self.slots.clear()

    def blocks_callback(self, msg: BlockArray) -> None:
        now = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if self.only_waiting_player and self.current_game_state != "WAITING_PLAYER":
            self.reset_missing_state(now)
            return

        observed = {}
        for block in msg.blocks:
            observed[block.id] = block

        if not self.slots_initialized:
            self.initialize_slots(observed, now)
            return

        for block_id, slot in self.slots.items():
            current = observed.get(block_id)
            slot_occupied = current is not None and self.matches_slot(slot, current)
            if slot_occupied:
                slot.last_seen = now
                slot.missing_since = None
                slot.selected_latched = False
                continue

            if slot.selected_latched:
                continue

            if slot.missing_since is None:
                slot.missing_since = now
                continue

            held_missing = (now - slot.missing_since).to_sec()
            cooled_down = (now - self.last_selection_time).to_sec() >= self.selection_cooldown_sec
            if held_missing >= self.missing_hold_sec and cooled_down:
                self.publish_selection(slot, now)
                slot.selected_latched = True

    def initialize_slots(self, observed: Dict[int, object], now: rospy.Time) -> None:
        self.slots.clear()
        for block_id, block in observed.items():
            self.slots[block_id] = SlotState(
                block_id=block_id,
                color=block.color,
                x=block.position.x,
                y=block.position.y,
                z=block.position.z,
                last_seen=now,
            )
        self.slots_initialized = bool(self.slots) and self.freeze_slots_on_first_frame

    def reset_missing_state(self, now: rospy.Time) -> None:
        for slot in self.slots.values():
            slot.missing_since = None
            slot.last_seen = now
            slot.selected_latched = False

    def matches_slot(self, slot: SlotState, block: object) -> bool:
        dx = slot.x - block.position.x
        dy = slot.y - block.position.y
        dz = slot.z - block.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= self.max_slot_distance_m

    def publish_selection(self, slot: SlotState, stamp: rospy.Time) -> None:
        msg = PlayerSelection()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame
        msg.block_id = slot.block_id
        msg.color = slot.color
        msg.selection_type = self.selection_type
        msg.selection_time = stamp
        msg.confidence = 1.0
        self.selection_pub.publish(msg)
        self.last_selection_time = stamp
        rospy.loginfo("Slot disappearance selection: block %d (%s)", slot.block_id, slot.color)


def main() -> None:
    rospy.init_node("player_selection_node")
    PlayerSelectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
