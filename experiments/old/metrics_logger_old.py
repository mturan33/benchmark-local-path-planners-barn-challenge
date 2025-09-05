#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import csv
import os
import math
import sys

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class MetricsLogger:
    def __init__(self):
        print("LOGGER BAŞLADI! (MetricsLogger __init__ girdi)")

        # ROS parametresinden ve komut satırından algorithm_name al
        algo_param = None
        try:
            algo_param = rospy.get_param("~algorithm_name")
        except Exception:
            pass

        # Komut satırı parametresi ile gelirse onu da al (_algorithm_name:=xyz)
        algo_arg = None
        for arg in sys.argv:
            if arg.startswith("_algorithm_name:="):
                algo_arg = arg.split(":=")[1]

        # Öncelik: Komut satırı > ROS parametresi > unknown
        self.algo = algo_arg or algo_param or "unknown"
        print("algorithm_name parametresi:", self.algo)

        rospy.init_node("metrics_logger", anonymous=True)

        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.collision_sub = rospy.Subscriber("/collision_occurred", Bool, self.collision_callback)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # Central metrics directory
        self.log_dir = os.path.expanduser(f"~/3v3/experiments/metrics")
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, f"metrics_log_{self.algo}.csv")
        self.init_csv()

        self.prev_x = None
        self.prev_y = None
        self.path_length = 0.0
        self.start_time = None
        self.end_time = None
        self.collision = False
        self.goal = None

    def init_csv(self):
        print("CSV INIT çalıştı!")
        if not os.path.exists(self.log_file):
            with open(self.log_file, "w", newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Time", "X", "Y",
                    "Path_Length", "Collision", 
                    "Start_Time", "End_Time", "AT"
                ])
            print(f"Başlangıçta boş CSV oluşturuldu: {self.log_file}")

    def goal_callback(self, msg):
        print("GOAL CALLBACK GELDİ!")
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.start_time = rospy.Time.now().to_sec()
        self.path_length = 0.0
        self.prev_x = None
        self.prev_y = None
        rospy.loginfo(f"[{self.algo.upper()} Logger] Goal received at ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def collision_callback(self, msg):
        print("COLLISION CALLBACK GELDİ!")
        if msg.data:
            self.collision = True
            rospy.logwarn(f"[{self.algo.upper()} Logger] Collision occurred!")

    def odom_callback(self, msg):
        print("ODOM CALLBACK GELDİ!")
        current_time = rospy.Time.now().to_sec()
        pos = msg.pose.pose.position
        x, y = pos.x, pos.y

        if self.prev_x is not None and self.prev_y is not None:
            dist = math.hypot(x - self.prev_x, y - self.prev_y)
            self.path_length += dist

        self.prev_x = x
        self.prev_y = y

        at = None
        if self.start_time is not None and self.goal is not None:
            dist_to_goal = math.hypot(self.goal[0] - x, self.goal[1] - y)
            if dist_to_goal < 0.25 and not self.collision:
                self.end_time = current_time
                at = self.end_time - self.start_time
                rospy.loginfo(f"[{self.algo.upper()} Logger] Goal reached. AT = {at:.2f} s")
                self.save_metrics(current_time, x, y, at)
                rospy.signal_shutdown("Goal reached or test done.")
                return

        self.save_metrics(current_time, x, y, at)

    def save_metrics(self, t, x, y, at):
        print(f"save_metrics çağrıldı! {self.log_file}")
        with open(self.log_file, "a", newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                round(t, 2), round(x, 2), round(y, 2),
                round(self.path_length, 2), int(self.collision),
                round(self.start_time or 0, 2),
                round(self.end_time or 0, 2),
                round(at, 2) if at else ""
            ])

if __name__ == "__main__":
    try:
        logger = MetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

