#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

import csv
import os
import math
import sys
import time

class MetricsLogger:
    def __init__(self):
        # Algoritma adını al
        algo_param = None
        try:
            algo_param = rospy.get_param("~algorithm_name")
        except Exception:
            pass
        algo_arg = None
        for arg in sys.argv:
            if arg.startswith("_algorithm_name:="):
                algo_arg = arg.split(":=")[1]
        self.algo = algo_arg or algo_param or "unknown"
        print(f"algorithm_name parametresi: {self.algo}")

        rospy.init_node("metrics_logger", anonymous=True)

        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.collision_sub = rospy.Subscriber("/collision", Bool, self.collision_callback)
        self.collision_sub2 = rospy.Subscriber("/collision_occurred", Bool, self.collision_callback)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.goal_sub2 = rospy.Subscriber("/move_base/goal", PoseStamped, self.goal_callback)
        self.goal_alt_sub = rospy.Subscriber("/move_base/goal", PoseStamped, self.goal_callback)


        # Output
        self.log_dir = os.path.expanduser("~/3v3/experiments/metrics")
        os.makedirs(self.log_dir, exist_ok=True)
        self.summary_file = os.path.join(self.log_dir, f"metrics_summary_{self.algo}.csv")
        self._init_summary_csv()

        # Metrik değişkenleri
        self.positions = []
        self.times = []
        self.path_length = 0.0
        self.start_time = None
        self.end_time = None
        self.collision = False
        self.goal = None
        self.goal_received = False

    def _init_summary_csv(self):
        if not os.path.exists(self.summary_file):
            with open(self.summary_file, "w", newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Algorithm", "World", "Trial",
                    "Total_Time", "Path_Length", "Deviation_From_Straight",
                    "Avg_Velocity", "Path_Efficiency", "Collision"
                ])
            print(f"Başlangıçta boş summary CSV oluşturuldu: {self.summary_file}")

    def goal_callback(self, msg):
        print("GOAL CALLBACK GELDİ!")
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.start_time = time.time()
        self.positions = []
        self.times = []
        self.path_length = 0.0
        self.goal_received = True
        self.collision = False
        rospy.loginfo(f"[{self.algo.upper()} Logger] Goal received at ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def collision_callback(self, msg):
        print("COLLISION CALLBACK GELDİ!")
        if msg.data:
            self.collision = True
            rospy.logwarn(f"[{self.algo.upper()} Logger] Collision occurred!")

    def odom_callback(self, msg):
        if not self.goal_received:
            return
        pos = msg.pose.pose.position
        now = time.time()
        self.positions.append((pos.x, pos.y))
        self.times.append(now)
        if len(self.positions) > 1:
            last_x, last_y = self.positions[-2]
            dist = math.hypot(pos.x - last_x, pos.y - last_y)
            self.path_length += dist

        # Hedefe varış
        if self.goal is not None:
            dist_to_goal = math.hypot(self.goal[0] - pos.x, self.goal[1] - pos.y)
            if dist_to_goal < 0.25 and not self.collision:
                self.end_time = now
                self._write_summary()
                rospy.signal_shutdown("Goal reached or test done.")

    def _write_summary(self):
        print("SUMMARY YAZILIYOR...")
        total_time = (self.end_time - self.start_time) if (self.start_time and self.end_time) else 0.0
        straight_line = math.hypot(self.goal[0] - self.positions[0][0], self.goal[1] - self.positions[0][1]) if self.goal else 0.0
        deviation_from_straight = (self.path_length / straight_line) if straight_line > 0 else 0.0
        avg_velocity = (self.path_length / total_time) if total_time > 0 else 0.0
        path_efficiency = (straight_line / self.path_length) if self.path_length > 0 else 0.0

        # World ve trial (run.py ile başlatırsan bunları arg olarak ekle, burada sabit "0" ve "1")
        world_idx = int(os.environ.get("WORLD_IDX", "0"))
        trial = int(os.environ.get("TRIAL", "1"))

        with open(self.summary_file, "a", newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.algo, world_idx, trial,
                round(total_time, 2), round(self.path_length, 2),
                round(deviation_from_straight, 3), round(avg_velocity, 2),
                round(path_efficiency, 3), int(self.collision)
            ])
        print(f"[✓] {self.summary_file} dosyasına satır eklendi.")

if __name__ == "__main__":
    try:
        logger = MetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

import glob
print("Var olan metrics dosyaları:", glob.glob(os.path.expanduser("~/3v3/experiments/metrics/*")))


