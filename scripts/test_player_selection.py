#!/usr/bin/env python
# Simple script to simulate player selection for testing
# Usage: rosrun memory_game test_player_selection.py

import rospy
from memory_game.msg import PlayerSelection
from std_msgs.msg import Header

def simulate_player_selection():
    rospy.init_node('test_player_selector', anonymous=True)
    pub = rospy.Publisher('/player_selection', PlayerSelection, queue_size=10)
    
    rospy.loginfo("Test player selection node started")
    rospy.loginfo("This will simulate player selecting blocks")
    rospy.loginfo("Wait for game to show sequence, then press Enter to select blocks")
    
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        try:
            # Wait for user input
            try:
                user_input = raw_input("Enter block ID to select (0-3), or 'q' to quit: ")
            except NameError:
                user_input = input("Enter block ID to select (0-3), or 'q' to quit: ")
            
            if user_input.lower() == 'q':
                break
            
            block_id = int(user_input)
            if block_id < 0 or block_id > 3:
                rospy.logwarn("Block ID must be 0-3")
                continue
            
            # Create selection message
            msg = PlayerSelection()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "panda_link0"
            msg.block_id = block_id
            msg.color = ["red", "green", "blue", "yellow"][block_id]
            msg.selection_type = "pointed"
            msg.selection_time = rospy.Time.now()
            msg.confidence = 0.9
            
            pub.publish(msg)
            rospy.loginfo("Published selection: Block %d (%s)", block_id, msg.color)
            
        except ValueError:
            rospy.logwarn("Invalid input, enter a number 0-3")
        except KeyboardInterrupt:
            break
    
    rospy.loginfo("Test player selection node shutting down")

if __name__ == '__main__':
    try:
        simulate_player_selection()
    except rospy.ROSInterruptException:
        pass
