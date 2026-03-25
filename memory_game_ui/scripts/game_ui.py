#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
import tkinter as tk
from queue import Queue

class GameUI:
    def __init__(self):
        # Internal state
        self.score = 0
        self.state = "IDLE"

        # Thread-safe queue for ROS → GUI communication
        self.queue = Queue()

        # ROS Subscribers
        rospy.Subscriber("/score", Int32, self.score_callback)
        rospy.Subscriber("/game_state", String, self.state_callback)

        # Create GUI
        self.root = tk.Tk()
        self.root.title("Memory Game Interface")
        self.root.geometry("400x200")

        self.score_label = tk.Label(
            self.root,
            text="Score: 0",
            font=("Arial", 32),
            fg="blue"
        )
        self.score_label.pack(pady=15)

        self.state_label = tk.Label(
            self.root,
            text="State: IDLE",
            font=("Arial", 18)
        )
        self.state_label.pack(pady=10)

        # Start update loop
        self.update_gui()

    # --- ROS callbacks (DO NOT touch GUI here) ---
    def score_callback(self, msg):
        self.queue.put(("score", msg.data))

    def state_callback(self, msg):
        self.queue.put(("state", msg.data))

    # --- GUI update loop ---
    def update_gui(self):
        while not self.queue.empty():
            msg_type, value = self.queue.get()

            if msg_type == "score":
                self.score = value
            elif msg_type == "state":
                self.state = value

        # Update labels safely in GUI thread
        self.score_label.config(text=f"Score: {self.score}")
        self.state_label.config(text=f"State: {self.state}")

        # Color feedback (nice touch for demo)
        if self.state == "GAME_OVER":
            self.state_label.config(fg="red")
        elif self.state == "WAITING_PLAYER":
            self.state_label.config(fg="orange")
        elif self.state == "SHOWING_SEQUENCE":
            self.state_label.config(fg="green")
        else:
            self.state_label.config(fg="black")

        # Run again after 100ms
        self.root.after(100, self.update_gui)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    rospy.init_node("game_ui", anonymous=True)

    ui = GameUI()
    ui.run()
