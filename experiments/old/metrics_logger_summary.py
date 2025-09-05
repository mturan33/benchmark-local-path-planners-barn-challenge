#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String
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
        self.recovery_sub = rospy.Subscriber("/move_base/recovery_status", String, self.recovery_callback)

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

        self.recovery_count = 0
        self.computation_times = []
        self.min_clearances = []
        self.max_path_deviation = 0.0
        self.visited_cells = set()  # Overlap
        self.overlap_count = 0

    def _init_summary_csv(self):
        if not os.path.exists(self.summary_file):
            with open(self.summary_file, "w", newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Algorithm", "World", "Trial",
                    "Total_Time", "Path_Length", "Deviation_From_Straight",
                    "Avg_Velocity", "Path_Efficiency", "Collision",
                    "Recovery_Count", "Avg_Computation_Time", "Path_Smoothness",
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
        self.computation_times = []
        self.min_clearances = []
        self.max_path_deviation = 0.0
        self.visited_cells = set()
        self.overlap_count = 0

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

        # Velocity (hız vektörü)
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

            # Path overlap: hücreyi işaretle
            cell = (round(pos.x, 2), round(pos.y, 2))
            if cell in self.visited_cells:
                self.overlap_count += 1
            self.visited_cells.add(cell)

        # Path deviation (maksimum)
        if self.goal is not None and self.positions:
            x0, y0 = self.positions[0]
            x1, y1 = self.goal
            px, py = pos.x, pos.y
            line_dist = self.point_line_distance(x0, y0, x1, y1, px, py)
            if line_dist > self.max_path_deviation:
                self.max_path_deviation = line_dist

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

    @staticmethod
    def point_line_distance(x0, y0, x1, y1, px, py):
        num = abs((y1 - y0)*px - (x1 - x0)*py + x1*y0 - y1*x0)
        den = math.hypot(y1 - y0, x1 - x0)
        return num / den if den > 0 else 0.0

    def _normalize(self, value, min_val, max_val, clamp_0_1=True):
        """Basit normalize fonksiyonu (0–1 arası)"""
        if max_val - min_val == 0:
            return 0.0
        norm = (value - min_val) / (max_val - min_val)
        if clamp_0_1:
            norm = min(max(norm, 0.0), 1.0)
        return norm

    def _write_summary(self):
        print("SUMMARY YAZILIYOR...")
        total_time = (self.end_time - self.start_time) if (self.start_time and self.end_time) else 0.0
        straight_line = math.hypot(self.goal[0] - self.positions[0][0], self.goal[1] - self.positions[0][1]) if self.goal and self.positions else 0.0
        deviation_from_straight = (self.path_length / straight_line) if straight_line > 0 else 0.0
        avg_velocity = (self.path_length / total_time) if total_time > 0 else 0.0
        path_efficiency = (straight_line / self.path_length) if self.path_length > 0 else 0.0

        # Path Smoothness
        smoothness = 0.0
        norm_smoothness = 0.0
        if len(self.angles) > 2:
            dtheta = np.diff(self.angles)
            dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
            smoothness = np.sum(np.abs(dtheta))
            # Normalize: 0 = çok düzgün, 1 = çok keskin
            norm_smoothness = self._normalize(smoothness, 0, 5 * np.pi)  # 5*pi rastgele, literature göre ayarla

        # Control effort (hız, ivme, açısal hız RMS toplamı)
        effort = 0.0
        norm_effort = 0.0
        jerk = 0.0
        norm_jerk = 0.0
        if len(self.velocities) > 2:
            vels = np.array(self.velocities)
            speeds = np.linalg.norm(vels, axis=1)
            acc = np.diff(speeds) / np.diff(self.times[:len(speeds)])
            effort = np.sqrt(np.mean(acc**2))
            if len(acc) > 1:
                jerk_vals = np.diff(acc) / np.diff(self.times[1:len(acc)+1])
                jerk = np.sqrt(np.mean(jerk_vals**2))
            # Normalize effort ve jerk, literature göre referans: 1 m/s2 ivme konforlu, jerk için 2 m/s3.
            norm_effort = self._normalize(effort, 0, 2)
            norm_jerk = self._normalize(jerk, 0, 5)

        # Path curvature
        curvature = 0.0
        norm_curvature = 0.0
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
            # Normalize: 0 = düz, 1 = çok kıvrık (referans: 10)
            norm_curvature = self._normalize(curvature, 0, 10)

        # Average computation time (örn. planlama döngüsünden alınıyorsa ekle)
        avg_comp_time = np.mean(self.computation_times) if self.computation_times else 0.0
        norm_comp_time = self._normalize(avg_comp_time, 0, 0.1)  # 0.1 sn üstü yavaş sayabilirsin

        # Minimum clearance (şu an boş, costmap ile entegre edebilirsin)
        min_clearance = min(self.min_clearances) if self.min_clearances else 0.0
        robot_radius = 0.21
        norm_clearance = self._normalize(min_clearance, robot_radius, 1.0)

        # Path overlap: toplam overlap'i path length'e oranla ver
        path_overlap = self.overlap_count
        norm_overlap = self._normalize(path_overlap, 0, self.path_length / 0.02 if self.path_length > 0 else 1)

        # Max path deviation (ideal path'ten max sapma)
        max_path_deviation = self.max_path_deviation
        norm_path_deviation = self._normalize(max_path_deviation, 0, straight_line)

        world_idx = int(os.environ.get("WORLD_IDX", "0"))
        trial = int(os.environ.get("TRIAL", "1"))

        with open(self.summary_file, "a", newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.algo, world_idx, trial,
                round(total_time, 2), round(self.path_length, 2),
                round(deviation_from_straight, 3), round(avg_velocity, 2),
                round(path_efficiency, 3), int(self.collision),
                self.recovery_count, round(norm_comp_time, 3), round(norm_smoothness, 3),
                round(norm_clearance, 3), round(norm_path_deviation, 3), round(norm_effort, 3),
                round(norm_curvature, 3), round(norm_jerk, 3), round(norm_overlap, 3)
            ])
        print(f"[✓] {self.summary_file} dosyasına normalize satır eklendi.")

if __name__ == "__main__":
    try:
        logger = MetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

