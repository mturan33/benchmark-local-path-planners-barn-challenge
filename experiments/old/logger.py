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
        rospy.init_node("metrics_logger", anonymous=True)

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

        # ENVIRONMENT değişkenlerinden world_idx ve trial'ı oku
        self.world_idx = int(os.environ.get("WORLD_IDX", "0"))
        self.trial = int(os.environ.get("TRIAL", "1"))

        self.log_dir = os.path.expanduser("~/3v3/experiments/metrics")
        os.makedirs(self.log_dir, exist_ok=True)
        # Her algoritma, dünya ve deneme için benzersiz bir dosya adı oluştur
        # auto_logger.py'nin bu dosyayı belirli bir isimle taşımasını beklediği için
        # şimdilik sadece algo.csv olarak kalacak, auto_logger.py zaten taşıma ve yeniden isimlendirme yapıyor.
        self.summary_file = os.path.join(self.log_dir, f"{self.algo}.csv")
        self.data_written = False # Verinin sadece bir kez yazıldığını kontrol etmek için

        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        self.collision_sub = rospy.Subscriber("/collision", Bool, self.collision_callback)
        self.goal_sub = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback)
        self.recovery_sub = rospy.Subscriber("/move_base/recovery_status", String, self.recovery_callback)

        self.start_time = None
        self.end_time = None
        self.robot_x = []
        self.robot_y = []
        self.goal_x = None
        self.goal_y = None
        self.collision = False
        self.recovery_count = 0
        self.my_recovery_count = 0
        self.last_recovery_time = 0
        self.recovery_interval = 2.0 # Minimum recovery mesajı aralığı

        self.path_length = 0.0
        self.min_clearances = []
        self.computation_times = []
        self.max_path_deviation = 0.0

        rospy.on_shutdown(self.write_summary) # ROS düğümü kapatılırken bu metodu çağır

        rospy.loginfo(f"Metrics Logger initialized for algorithm: {self.algo}, World: {self.world_idx}, Trial: {self.trial}")

    def odom_callback(self, msg):
        if self.start_time is None:
            self.start_time = rospy.get_time()

        self.robot_x.append(msg.pose.pose.position.x)
        self.robot_y.append(msg.pose.pose.position.y)

        # Basit yol uzunluğu hesaplaması
        if len(self.robot_x) > 1:
            dx = self.robot_x[-1] - self.robot_x[-2]
            dy = self.robot_y[-1] - self.robot_y[-2]
            self.path_length += math.sqrt(dx*dx + dy*dy)
        
        # Olası costmap_clearance veya benzeri metrikler için subscriber
        # Eğer varsa bu bilgiyi buradan toplayın. Yoksa min_clearance -1 kalacaktır.
        # Örneğin: rospy.Subscriber("/robot_clearance", Float32, self.clearance_callback)

    def collision_callback(self, msg):
        if msg.data:
            self.collision = True
            rospy.logwarn("Collision detected!")

    def goal_callback(self, msg):
        # Hedef koordinatları alındığında
        if self.goal_x is None: # İlk hedef alındığında
            self.goal_x = msg.goal.pose.position.x
            self.goal_y = msg.goal.pose.position.y
            rospy.loginfo(f"Goal set to: ({self.goal_x}, {self.goal_y})")
        # Hedefe ulaşıldığında (veya yeni hedef verildiğinde) end_time'ı ayarla
        # Hedefe ulaşma mantığı burada daha karmaşık olabilir (örn. move_base'in durumu)
        # Şimdilik basitçe ilk hedef alındığında başlat, herhangi bir "başarılı" durumu bitiş kabul et.
        # Bu logic'i, hedefe ulaşıldığında çağrılacak ayrı bir callback ile iyileştirebilirsiniz.
        # Örneğin: move_base/status mesajlarına abone olmak.
        # Şimdilik, eğer robot hedef bölgesine yaklaştıysa veya move_base başarılı statüsü dönerse end_time'ı ayarlayın.
        # Basitçe, eğer goal_x ve goal_y zaten ayarlanmışsa ve yeni bir goal mesajı geliyorsa,
        # bu yeni bir denemenin başlangıcı olabilir veya mevcut denemenin bitişi olarak kabul edilebilir.
        # Otomatik tester senaryosunda, genellikle tek bir hedef vardır ve ona ulaşınca biter.
        # Bu kısım, move_base'in terminal durumlarını dinleyerek daha doğru yönetilebilir.
        pass # Bu kısım şimdilik doğrudan end_time'ı set etmiyor

    def recovery_callback(self, msg):
        current_time = rospy.get_time()
        # Çok yakın zamanlarda gelen recovery mesajlarını filtrele
        if current_time - self.last_recovery_time > self.recovery_interval:
            self.recovery_count += 1
            if "my_recovery_message" in msg.data.lower(): # Özel kurtarma mesajı varsa
                self.my_recovery_count += 1
            rospy.loginfo(f"Recovery message: {msg.data} (Total: {self.recovery_count})")
            self.last_recovery_time = current_time

    def write_summary(self):
        # Bu metod ROS düğümü kapanırken otomatik çağrılacak.
        # Veriyi sadece bir kez yazdığından emin olmak için bayrak kullanıyoruz.
        if self.data_written:
            rospy.loginfo("Metrics already written, skipping.")
            return

        rospy.loginfo("Writing summary metrics...")
        self.end_time = rospy.get_time() # Kapanış anı bitiş zamanı olarak kabul edilsin

        # Hesaplamalar
        total_time = (self.end_time - self.start_time) if self.start_time and self.end_time else 0.0

        # Path Length zaten odom_callback'te toplanıyor

        # Deviation_From_Straight (Düzden Sapma)
        deviation_from_straight = 0.0
        if self.goal_x is not None and len(self.robot_x) > 1:
            # Hedef noktasına olan düz çizgi
            start_x, start_y = self.robot_x[0], self.robot_y[0]
            end_x, end_y = self.goal_x, self.goal_y
            
            if start_x != end_x or start_y != end_y:
                line_vec_x = end_x - start_x
                line_vec_y = end_y - start_y
                line_length_sq = line_vec_x**2 + line_vec_y**2
                
                max_deviation = 0.0
                for i in range(len(self.robot_x)):
                    p_x, p_y = self.robot_x[i], self.robot_y[i]
                    # Noktanın doğruya dik uzaklığı
                    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
                    distance = abs(line_vec_y * p_x - line_vec_x * p_y + end_x * start_y - end_y * start_x) / math.sqrt(line_length_sq)
                    if distance > max_deviation:
                        max_deviation = distance
                deviation_from_straight = max_deviation
            else: # Başlangıç ve bitiş noktası aynıysa
                deviation_from_straight = 0.0
        
        self.max_path_deviation = deviation_from_straight # max_path_deviation da bu metrik olabilir.

        # Avg_Velocity
        avg_velocity = self.path_length / total_time if total_time > 0 else 0.0

        # Path_Efficiency (Yol verimliliği)
        ideal_dist = 0.0
        if self.goal_x is not None and len(self.robot_x) > 0:
            ideal_dist = math.sqrt((self.goal_x - self.robot_x[0])**2 + (self.goal_y - self.robot_y[0])**2)
        path_efficiency = ideal_dist / self.path_length if self.path_length > 0 else 0.0

        # Smoothness (Yol Düzgünlüğü) - Curvature (Eğrilik) ve Jerk (Sarsıntı) ile bağlantılı
        # Detaylı hesaplama için robot_x, robot_y verisi yetmeyebilir, robot_vel ve robot_accel da gerekebilir.
        # Basit bir yaklaşımla, hız vektörlerindeki değişim üzerinden hesaplanabilir.
        smoothness = 0.0
        curvature = 0.0
        jerk = 0.0 # Daha ileri bir türev

        if len(self.robot_x) > 2:
            # Numerik türevler
            dx = np.gradient(np.array(self.robot_x))
            dy = np.gradient(np.array(self.robot_y))

            # Hız vektörleri
            vx = dx / (rospy.get_time() - self.start_time) # Basit bir zaman ölçeklendirmesi
            vy = dy / (rospy.get_time() - self.start_time)
            
            # Eğrilik (Curvature)
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)
            
            num = dx * ddy - dy * ddx
            denom = (dx**2 + dy**2)**1.5
            
            valid = denom != 0 # Payda sıfır olmayan yerler
            curvatures = np.zeros_like(num)
            curvatures[valid] = np.abs(num[valid] / denom[valid])
            curvature = np.mean(curvatures) if len(curvatures[valid]) > 0 else 0.0
            
            # Jerk (Sarsıntı)
            dddx = np.gradient(ddx)
            dddy = np.gradient(ddy)
            jerk = np.mean(np.sqrt(dddx**2 + dddy**2)) if len(dddx) > 0 else 0.0
            
            # Smoothness için farklı metotlar kullanılabilir (örn. toplam açısal değişim)
            # Şimdilik curvature'ı bir proxy olarak kullanalım veya farklı bir metrik tanımlayalım
            smoothness = -curvature # Negatif eğrilik, daha küçük eğrilik daha düzgün anlamına gelir

        # Effort: Robotun harcadığı enerji veya kontrol çabası.
        # Genellikle motor torku/akımından veya komut hızının toplamından hesaplanır.
        # Bu loggerda o bilgi yok, varsayılan olarak 0 veya N/A bırakılabilir.
        effort = 0.0

        # Min_Clearance: collision_sub ile toplanan bilgi yok, varsa buradan gelecek
        # Bu kısım için ayrı bir subscriber ve logic kurmanız gerekir.
        min_clearance = min(self.min_clearances) if self.min_clearances else -1.0 # -1 değeri veri yok anlamında

        # Overlap_Count: Henüz toplanmıyor, bu metrik için ayrı bir logic lazım.
        overlap_count = 0

        # CSV'ye yaz
        try:
            # Dosyayı her zaman yeniden yaz (append değil), çünkü auto_logger.py her seferinde bu dosyayı taşıyacak
            # ve bizden tek bir satır veri bekleyecek.
            with open(self.summary_file, "w", newline='') as file:
                writer = csv.writer(file)
                # Başlıkları sadece bir kez yazmak istiyorsak buraya if logic eklenebilir,
                # ama auto_logger.py'nin zaten bunları özet dosyaya eklediği için buna gerek yok.
                writer.writerow([
                    self.algo, self.world_idx, self.trial,
                    round(total_time, 2), round(self.path_length, 2),
                    round(deviation_from_straight, 3), round(avg_velocity, 2),
                    round(path_efficiency, 3), int(self.collision),
                    self.recovery_count, self.my_recovery_count, round(avg_comp_time, 3), round(smoothness, 3),
                    round(min_clearance, 3), round(self.max_path_deviation, 3), round(effort, 3),
                    round(curvature, 3), round(jerk, 3), overlap_count
                ])
            rospy.loginfo(f"Metrikler başarıyla kaydedildi: {self.summary_file}")
            self.data_written = True # Veri yazıldı olarak işaretle

            # DEBUG: auto_logger.py'nin yakalayabileceği formatta konsola bas
            # Bu çıktı, auto_logger.py'deki extract_navigation_metric tarafından kullanılacak
            csv_output_data = [
                self.algo, self.world_idx, self.trial,
                round(total_time, 2), round(self.path_length, 2),
                round(deviation_from_straight, 3), round(avg_velocity, 2),
                round(path_efficiency, 3), int(self.collision),
                self.recovery_count, self.my_recovery_count, round(avg_comp_time, 3), round(smoothness, 3),
                round(min_clearance, 3), round(self.max_path_deviation, 3), round(effort, 3),
                round(curvature, 3), round(jerk, 3), overlap_count
            ]
            # Liste elemanlarını tırnak içinde ve virgülle ayırarak stringe dönüştür
            print(f"CSV Output: {csv_output_data}")

        except Exception as e:
            rospy.logerr(f"Metrikler yazılırken hata oluştu: {e}")
            self.data_written = False # Hata olursa tekrar yazmayı denemeye açık bırak

if __name__ == '__main__':
    try:
        metrics_logger = MetricsLogger()
        rospy.spin() # ROS düğümünün aktif kalmasını sağlar
    except rospy.ROSInterruptException:
        rospy.loginfo("Metrics Logger interrupted.")
    finally:
        # rospy.on_shutdown zaten çağrılacağı için burada tekrar manuel çağırmaya gerek yok.
        # rospy.spin() kesildiğinde veya ROS kapatıldığında write_summary otomatik çağrılacaktır.
        pass
