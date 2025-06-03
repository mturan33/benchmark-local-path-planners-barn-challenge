#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String  # Recovery için
import csv
import os
import math
import sys
import time
import numpy as np

class MetricsLogger:
    def __init__(self):
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
        self.goal_sub = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback)

        # Recovery callback (move_base’den gelen)
        self.recovery_sub = rospy.Subscriber("/move_base/recovery_status", String, self.recovery_callback)  # recovery status mesajı

        self.log_dir = os.path.expanduser("~/3v3/experiments/metrics")
        os.makedirs(self.log_dir, exist_ok=True)
        self.summary_file = os.path.join(self.log_dir, f"metrics_summary_{self.algo}.csv")
        self._init_summary_csv()

        self.positions = []
        self.times = []
        self.velocities = []
        self.angles = []
        self.path_length = 0.0
        self.start_time = None
        self.end_time = None
        self.collision = False
        self.goal = None
        self.goal_received = False

        self.recovery_count = 0  # move_base’den gelen
        self.my_recovery_count = 0  # kendi mantığımızla hesaplanan
        self.computation_times = []
        self.min_clearances = []
        self.max_path_deviation = 0.0
        self.visited_cells = set()  # Overlap
        self.overlap_count = 0

        # Kendi recovery sayaç parametreleri
        self.last_progress_time = None
        self.last_position = None
        self.no_progress_threshold = rospy.get_param("~no_progress_threshold", 2.0)  # sn
        self.progress_epsilon = rospy.get_param("~progress_epsilon", 0.05)  # metre (5 cm)

        # Timeout parametresi
        self.timeout_seconds = rospy.get_param("~timeout_seconds", 150)

    def _init_summary_csv(self):
        if not os.path.exists(self.summary_file):
            with open(self.summary_file, "w", newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Algorithm", "World", "Trial",
                    "Total_Time", "Path_Length", "Deviation_From_Straight",
                    "Avg_Velocity", "Path_Efficiency", "Collision",
                    "Recovery_Count", "My_Recovery_Count", "Avg_Computation_Time", "Path_Smoothness",
                    "Min_Clearance", "Max_Path_Deviation", "Control_Effort",
                    "Path_Curvature", "Avg_Jerk", "Path_Overlap"
                ])
            print(f"Başlangıçta boş summary CSV oluşturuldu: {self.summary_file}")

    def goal_callback(self, msg):
        print("GOAL CALLBACK GELDİ!")
        self.goal = (msg.goal.target_pose.pose.position.x, msg.goal.target_pose.pose.position.y)
        print(f"Hedef: {self.goal}")
        self.start_time = time.time()
        self.positions = []
        self.times = []
        self.velocities = []
        self.angles = []
        self.path_length = 0.0
        self.goal_received = True
        self.collision = False
        self.recovery_count = 0
        self.my_recovery_count = 0
        self.computation_times = []
        self.min_clearances = []
        self.max_path_deviation = 0.0
        self.visited_cells = set()
        self.overlap_count = 0
        self.last_progress_time = None
        self.last_position = None

        rospy.loginfo(f"[{self.algo.upper()} Logger] Goal received at ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def collision_callback(self, msg):
        if self.goal_received:
            print("COLLISION CALLBACK GELDİ!")
            if msg.data:
                self.collision = True
                rospy.logwarn(f"[{self.algo.upper()} Logger] Collision occurred!")

    def recovery_callback(self, msg):
        self.recovery_count += 1
        rospy.logwarn(f"[{self.algo.upper()} Logger] Recovery triggered! (Toplam: {self.recovery_count})")

    def odom_callback(self, msg):
        if not self.goal_received:
            return
        pos = msg.pose.pose.position
        now = time.time()
        self.positions.append((pos.x, pos.y))
        self.times.append(now)

        # Velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.velocities.append((vx, vy))

        # Açısal yön
        if len(self.positions) > 1:
            dx = pos.x - self.positions[-2][0]
            dy = pos.y - self.positions[-2][1]
            angle = math.atan2(dy, dx)
            self.angles.append(angle)
            dist = math.hypot(dx, dy)
            self.path_length += dist

            # Path overlap
            cell = (round(pos.x, 2), round(pos.y, 2))
            if cell in self.visited_cells:
                self.overlap_count += 1
            self.visited_cells.add(cell)

        # Path deviation max
        if self.goal is not None and self.positions:
            x0, y0 = self.positions[0]
            x1, y1 = self.goal
            px, py = pos.x, pos.y
            line_dist = self.point_line_distance(x0, y0, x1, y1, px, py)
            if line_dist > self.max_path_deviation:
                self.max_path_deviation = line_dist

        # --- Kendi recovery sayaç kontrolü ---
        progress = False
        if self.last_position is not None:
            dx = pos.x - self.last_position[0]
            dy = pos.y - self.last_position[1]
            moved = math.hypot(dx, dy)
            if moved > self.progress_epsilon:  # eşiği ayarla, default 5cm
                progress = True
        else:
            progress = True  # ilk pozisyon

        if progress:
            self.last_progress_time = now
            self.last_position = (pos.x, pos.y)
        else:
            if self.last_progress_time is not None and (now - self.last_progress_time) > self.no_progress_threshold:
                self.my_recovery_count += 1
                rospy.logwarn(f"[{self.algo.upper()} Logger] [Kendi Recovery] {self.no_progress_threshold} sn hareketsizlik! (Toplam: {self.my_recovery_count})")
                self.last_progress_time = now  # birden fazla sayaç artmasın diye

        # Hedefe varış
        if self.goal is not None:
            dist_to_goal = math.hypot(self.goal[0] - pos.x, self.goal[1] - pos.y)
            if dist_to_goal < 2.0 and not self.collision:
                self.end_time = now
                self._write_summary()
                rospy.signal_shutdown("Goal reached or test done.")
                return
            elif self.collision:
                self.end_time = now
                self._write_summary()
                rospy.signal_shutdown("Collision or test done.")
                return

        # ------------- Timeout kontrolü ------------------
        if self.start_time is not None and (now - self.start_time) > self.timeout_seconds:
            self.end_time = now
            print(f"Timeout ({self.timeout_seconds} saniye) doldu, log kaydediliyor...")
            self._write_summary()
            rospy.signal_shutdown("Timeout doldu, test kapandı.")
            return
        # ---------------------------------------------------------

    @staticmethod
    def point_line_distance(x0, y0, x1, y1, px, py):
        num = abs((y1 - y0)*px - (x1 - x0)*py + x1*y0 - y1*x0)
        den = math.hypot(y1 - y0, x1 - x0)
        return num / den if den > 0 else 0.0

    def _write_summary(self):
        print("SUMMARY YAZILIYOR...")
        total_time = (self.end_time - self.start_time) if (self.start_time and self.end_time) else 0.0
        straight_line = math.hypot(self.goal[0] - self.positions[0][0], self.goal[1] - self.positions[0][1]) if self.goal and self.positions else 0.0
        deviation_from_straight = (self.path_length / straight_line) if straight_line > 0 else 0.0
        avg_velocity = (self.path_length / total_time) if total_time > 0 else 0.0
        path_efficiency = (straight_line / self.path_length) if self.path_length > 0 else 0.0

        # Path Smoothness: ardışık açı değişimlerinin toplamı
        smoothness = 0.0
        if len(self.angles) > 2:
            dtheta = np.diff(self.angles)
            dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
            smoothness = np.sum(np.abs(dtheta))

        # Control effort (hız, ivme, açısal hız RMS toplamı)
        effort = 0.0
        jerk = 0.0
        if len(self.velocities) > 2:
            vels = np.array(self.velocities)
            speeds = np.linalg.norm(vels, axis=1)
            acc = np.diff(speeds) / np.diff(self.times[:len(speeds)])
            effort = np.sqrt(np.mean(acc**2))
            if len(acc) > 1:
                jerk_vals = np.diff(acc) / np.diff(self.times[:len(acc)])
                jerk = np.sqrt(np.mean(jerk_vals**2))

        # Path curvature
        curvature = 0.0
        if len(self.positions) > 2:
            pos = np.array(self.positions)
            dx = np.gradient(pos[:, 0])
            dy = np.gradient(pos[:, 1])
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)
            num = dx * ddy - dy * ddx
            denom = (dx**2 + dy**2)**1.5
            valid = denom > 0
            curvatures = np.zeros_like(num)
            curvatures[valid] = np.abs(num[valid] / denom[valid])
            curvature = np.sum(curvatures)

        avg_comp_time = np.mean(self.computation_times) if self.computation_times else 0.0
        min_clearance = min(self.min_clearances) if self.min_clearances else -1

        world_idx = int(os.environ.get("WORLD_IDX", "0"))
        trial = int(os.environ.get("TRIAL", "1"))

        with open(self.summary_file, "a", newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.algo, world_idx, trial,
                round(total_time, 2), round(self.path_length, 2),
                round(deviation_from_straight, 3), round(avg_velocity, 2),
                round(path_efficiency, 3), int(self.collision),
                self.recovery_count, self.my_recovery_count, round(avg_comp_time, 3), round(smoothness, 3),
                round(min_clearance, 3), round(self.max_path_deviation, 3), round(effort, 3),
                round(curvature, 3), round(jerk, 3), self.overlap_count
            ])
            file.flush()
            os.fsync(file.fileno())
        print(f"[✓] {self.summary_file} dosyasına satır eklendi.")

if __name__ == "__main__":
    try:
        logger = MetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

