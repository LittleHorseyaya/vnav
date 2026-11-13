#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tkinter as tk
import time

# Gate 坐标
FIRST_GATE_X = 0.42
FIRST_GATE_Y = 3.17
LAST_GATE_X = -39.62
LAST_GATE_Y = 14.97
START_X = 0.40
START_Y = 8.00
GATE_THRESHOLD = 3.0
START_THRESHOLD = 5.0

class RaceTimerGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Race Timer")
        self.root.geometry("300x150+1500+100")
        self.root.attributes('-topmost', True)
        
        self.time_label = tk.Label(self.root, text="0.00s", font=("Arial", 48, "bold"))
        self.time_label.pack(expand=True)
        
        self.status_label = tk.Label(self.root, text="等待起飞...", font=("Arial", 14))
        self.status_label.pack()
        
        self.start_time = None
        self.elapsed = 0.0
        self.running = False
        self.passed_last = False
        self.taken_off = False
        self.at_start = True
        
        rospy.Subscriber('/current_state', Odometry, self.state_callback)
        
        self.update_timer()
        
    def state_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        dist_to_start = ((x - START_X)**2 + (y - START_Y)**2)**0.5
        print(f"位置：({x:.2f}, {y:.2f}, {z:.2f}), 距起点：{dist_to_start:.2f}m, at_start={self.at_start}, passed_last={self.passed_last}")
        
        # 检测复位（回到起点）
        if dist_to_start < START_THRESHOLD and (self.passed_last or self.taken_off):
           print("检测到复位！")
           self.at_start = True
        
        # 检测复位（回到起点）
        if dist_to_start < START_THRESHOLD and (self.passed_last or self.taken_off):
            self.at_start = True
            self.taken_off = False
            self.passed_last = False
            self.running = False
            self.elapsed = 0.0
            self.time_label.config(text="0.00s")
            self.status_label.config(text="等待起飞...")
            return
        
        # 检测起飞
        if not self.taken_off and z > 1.0:
            self.taken_off = True
            self.at_start = False
            self.start_time = time.time()
            self.running = True
            self.status_label.config(text="计时中...")
        
        # 检测穿过最后一个 gate
        if self.taken_off and not self.passed_last:
            dist = ((x - LAST_GATE_X)**2 + (y - LAST_GATE_Y)**2)**0.5
            if dist < GATE_THRESHOLD:
                self.passed_last = True
                self.running = False
                self.elapsed = time.time() - self.start_time
                self.time_label.config(text=f"{self.elapsed:.2f}s")
                self.status_label.config(text="完成！按 R 复位")
    
    def update_timer(self):
        if self.running:
            self.elapsed = time.time() - self.start_time
            self.time_label.config(text=f"{self.elapsed:.2f}s")
        self.root.after(100, self.update_timer)
    
    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    rospy.init_node('race_timer_gui', anonymous=True)
    gui = RaceTimerGUI()
    gui.run()
