#!/usr/bin/env python3
# Simple script to simulate player selection for testing
# Usage: rosrun memory_game test_player_selection.py

import rospy
from memory_game.msg import PlayerSelection
from std_msgs.msg import Header


COLOR_BY_ID = {
    0: "red",
    1: "green",
    2: "blue",
    3: "yellow",
}


def publish_selection(pub, block_id):
    msg = PlayerSelection()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "panda_link0"
    msg.block_id = block_id
    msg.color = COLOR_BY_ID.get(block_id, "unknown")
    msg.selection_type = "pointed"
    msg.selection_time = rospy.Time.now()
    msg.confidence = 0.9
    pub.publish(msg)
    rospy.loginfo("Published selection: Block %d (%s)", block_id, msg.color)


def simulate_player_selection():
    rospy.init_node('test_player_selector', anonymous=True)
    pub = rospy.Publisher('/player_selection', PlayerSelection, queue_size=10)

    rospy.loginfo("Test player selection node started")
    rospy.loginfo("Type one block id (e.g. 2) or sequence (e.g. 0 1 3). 'q' quits.")

    # Give publisher time to register before first manual input.
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        try:
            user_input = input("Select block(s): ").strip()
            if user_input.lower() == 'q':
                break

            if not user_input:
                continue

            ids = [int(tok) for tok in user_input.split()]
            for block_id in ids:
                if block_id < 0:
                    rospy.logwarn("Block ID must be >= 0")
                    continue
                publish_selection(pub, block_id)
                rospy.sleep(0.05)

        except ValueError:
            rospy.logwarn("Invalid input. Use integers like: 2 0 1")
        except KeyboardInterrupt:
            break

    rospy.loginfo("Test player selection node shutting down")

if __name__ == '__main__':
    try:
        simulate_player_selection()
    except rospy.ROSInterruptException:
        pass
