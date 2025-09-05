#!/usr/bin/env python3

import os
import subprocess
import time
import argparse
import socket
import csv
import re
import shutil
import signal # 'signal' modülü eklendi!

# --- Sabit Tanımlamaları ---
# Algoritmaların tanımları: her algoritma için sanal ortam, çalışma alanı ve ana script
# Sanal ortamlar ~/3v3/venvs/ altında
# Çalışma alanları ~/3v3/ altında
ALGORITHMS = {
    "dwa": {
        "venv": "barn_dwa",
        "ws": "ws_barn_dwa_eband_teb",
        "script": "run_dwa.py"
    },
    "fastdwa": {
        "venv": "barn_fastdwa",
        "ws": "ws_barn_fastdwa",
        "script": "run.py"
    },
    "eband": {
        "venv": "barn_eband",
        "ws": "ws_barn_dwa_eband_teb",
        "script": "run_eband.py"
    },
    "teb": {
        "venv": "barn_teb",
        "ws": "ws_barn_dwa_eband_teb",
        "script": "run_teb.py"
    },
    "sac": {
        "venv": "barn_sac",
        "ws": "ws_barn_sac",
        "script": "run.py"
    },
    "lfh": {
        "venv": "barn_lfh",
        "ws": "ws_barn_lfh",
        "script": "run.py"
    },
    "applr": {
        "venv": "barn_applr",
        "ws": "ws_barn_applr",
        "script": "run.py"
    },
    "e2e": {
        "venv": "barn_e2e",
        "ws": "ws_barn_e2e",
        "script": "run.py"
    },
    "mpc": {
        "venv": "barn_mpc",
        "ws": "ws_barn_mpc",
        "script": "run.py"
    },
    "neo": {
        "venv": "barn_neo",
        "ws": "ws_barn_neo",
        "script": "run.py"
    },
    "diff": {
        "venv": "barn_diff",
        "ws": "ws_barn_diff",
        "script": "run.py"
    },
    "purepursuit": {
        "venv": "barn_purepursuit",
        "ws": "ws_barn_purepursuit",
        "script": "run.py"
    },
    "dyna": {
        "venv": "barn_dyna",
        "ws": "ws_barn_dyna",
        "script": "run.py"
    },
    "mtube": {
        "venv": "barn_mtube",
        "ws": "ws_barn_lfh",
        "script": "run.py"
    }
}


# Log ve sonuç dosyalarının saklanacağı ana dizin
EXPERIMENTS_DIR = os.path.expanduser("~/3v3/experiments")
LOGS_DIR = os.path.join(EXPERIMENTS_DIR, "logs")
SUMMARY_CSV_PATH = os.path.join(EXPERIMENTS_DIR, "summary_results.csv")

# CSV başlıkları (logger.py'deki sıraya göre güncellenmeli)
CSV_HEADERS = [
    "Algorithm", "World", "Trial", "Total_Time", "Path_Length",
    "Deviation_From_Straight", "Avg_Velocity", "Path_Efficiency", "Collision",
    "Recovery_Count", "My_Recovery_Count", "Avg_Comp_Time", "Smoothness",
    "Min_Clearance", "Max_Path_Deviation", "Effort", "Curvature", "Jerk", "Overlap_Count"
]

# --- Yardımcı Fonksiyonlar ---

def is_roscore_running():
    """roscore'un çalışıp çalışmadığını kontrol eder."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(1)
        s.connect(("localhost", 11311))
        s.close()
        return True
    except Exception:
        return False

def start_roscore():
    """roscore'u başlatır."""
    print("[+] roscore çalışmıyor, başlatılıyor...")
    proc = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL, # stdout ve stderr'i gizle
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid # Child process'in kendi process group'unda çalışmasını sağlar
    )
    for _ in range(10): # roscore'un başlaması için 10 saniyeye kadar bekle
        if is_roscore_running():
            print("[✓] roscore başarıyla başlatıldı!")
            return proc
        time.sleep(1)
    print("[!] roscore başlatılamadı, elle kontrol edin!")
    return None

def setup_environment(algorithm_name, config):
    """Belirtilen algoritma için sanal ortamı etkinleştirir ve çalışma dizinine geçer."""
    # Sanal ortamlar ~/3v3/venvs/ altında bulunuyor
    venv_path = os.path.expanduser(f"~/3v3/venvs/{config['venv']}")
    # Çalışma alanları ~/3v3/ altında bulunuyor
    ws_path = os.path.expanduser(f"~/3v3/{config['ws']}")

    source_setup_path = os.path.join(ws_path, "devel", "setup.bash")

    if not os.path.exists(venv_path):
        print(f"[!] Sanal ortam bulunamadı: {venv_path}. Lütfen yolu kontrol edin ve doğru olduğundan emin olun.")
        return None, None
    if not os.path.exists(source_setup_path):
        print(f"[!] ROS çalışma alanı kurulum dosyası bulunamadı: {source_setup_path}. Lütfen yolu kontrol edin.")
        return None, None

    cmd_prefix = f"source {venv_path}/bin/activate && source {source_setup_path} && "
    return cmd_prefix, ws_path

def create_summary_csv():
    """Sonuçların yazılacağı ana CSV dosyasını oluşturur veya başlığını yazar."""
    if not os.path.exists(SUMMARY_CSV_PATH):
        os.makedirs(os.path.dirname(SUMMARY_CSV_PATH), exist_ok=True)
        with open(SUMMARY_CSV_PATH, "w", newline='') as file:
            writer = csv.writer(file)
            writer.writerow(CSV_HEADERS)
        print(f"[+] Yeni özet CSV dosyası oluşturuldu: {SUMMARY_CSV_PATH}")

def extract_navigation_metric(result_txt_path):
    """Verilen log dosyasından navigasyon metriklerini çeker."""
    if not os.path.exists(result_txt_path):
        print(f"[!] Log dosyası bulunamadı: {result_txt_path}. Metrikler çekilemedi.")
        return None

    with open(result_txt_path, "r") as f:
        content = f.read()

    # logger.py'nin konsola bastığı CSV Output satırını ara
    match = re.search(r"CSV Output:\s*\[(.*?)\]", content)
    if match:
        csv_str = match.group(1)
        # Tırnakları temizle ve virgülle ayırarak listeye dönüştür
        # `ast.literal_eval` daha güvenli bir yol olabilir ancak basit virgülle ayırma da işe yarar
        metrics = [item.strip().strip("'") for item in csv_str.split(',')]
        return metrics
    print(f"[!] '{result_txt_path}' dosyasından 'CSV Output' satırı bulunamadı.")
    return None

def add_navigation_metric_to_csv(metric_data):
    """Çıkarılan metrikleri özet CSV dosyasına ekler."""
    if metric_data: # Metrik veri boş değilse yaz
        with open(SUMMARY_CSV_PATH, "a", newline='') as file:
            writer = csv.writer(file)
            writer.writerow(metric_data)
    else:
        print("[!] Boş metrik verisi, özet CSV'ye eklenmedi.")

# --- Ana Çalıştırma Mantığı ---

def run_single_experiment(algo_name, config, world_idx, trial_num):
    """Tek bir algoritma, dünya ve deneme kombinasyonunu çalıştırır."""
    print(f"\n--- Çalıştırılıyor: {algo_name.upper()} | World: {world_idx} | Deneme: {trial_num} ---")

    # Log dosyaları için yol ve isimler
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_file_prefix = f"{algo_name}_world{world_idx}_trial{trial_num}_{timestamp}"
    result_txt_file = os.path.join(LOGS_DIR, f"{log_file_prefix}.txt")
    metrics_csv_file = os.path.join(LOGS_DIR, f"{log_file_prefix}_metrics.csv")

    os.makedirs(LOGS_DIR, exist_ok=True)

    # Ortam değişkenlerini ayarla ki logger.py okuyabilsin
    os.environ["WORLD_IDX"] = str(world_idx)
    os.environ["TRIAL"] = str(trial_num)

    cmd_prefix, work_dir = setup_environment(algo_name, config)
    if cmd_prefix is None:
        print(f"[-] Ortam kurulamadı, {algo_name} | World: {world_idx} | Deneme: {trial_num} atlanıyor.")
        return

    metrics_proc = None
    try:
        # 1. metrics_logger.py'yi başlat
        metrics_logger_cmd = f"rosrun barn_experiments logger.py _algorithm_name:={algo_name}"
        metrics_proc = subprocess.Popen(
            cmd_prefix + metrics_logger_cmd,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=work_dir,
            preexec_fn=os.setsid
        )
        print("[+] metrics_logger.py başlatıldı.")
        time.sleep(2) # logger'ın başlaması için kısa bir bekleme

        # 2. Asıl algoritma scriptini çalıştır
        main_script_cmd = f"roslaunch barn_setup barn_run.launch algorithm_name:={algo_name} world_idx:={world_idx}"
        print(f"[+] Çalıştırılan komut: {main_script_cmd}")

        main_process_result = subprocess.run(
            cmd_prefix + main_script_cmd,
            shell=True,
            executable="/bin/bash",
            cwd=work_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=300 # 5 dakika timeout
        )
        output = main_process_result.stdout.decode("utf-8", errors='ignore') # Hatalı karakterleri göz ardı et
        with open(result_txt_file, "w") as f:
            f.write(output)
        print(f"[✓] Ana script tamamlandı, log: {result_txt_file}")

    except subprocess.TimeoutExpired:
        print(f"[!] {algo_name.upper()} TIMEOUT! Log: {result_txt_file}")
        with open(result_txt_file, "a") as f:
            f.write("\n⏱ TIMEOUT\n")
    except Exception as e:
        print(f"[!] Ana script çalıştırılırken hata oluştu: {e}")
        with open(result_txt_file, "a") as f:
            f.write(f"\nError: {e}\n")
    finally:
        metrics_outs = b''
        metrics_errs = b''

        # metrics_logger.py process'ini sonlandır ve çıktılarını al
        if metrics_proc and metrics_proc.poll() is None:
            try:
                # communicate process'in bitmesini bekler ve çıktıyı alır.
                # Eğer process kapanmakta zorlanırsa timeout'a düşer.
                metrics_outs, metrics_errs = metrics_proc.communicate(timeout=10)
                print("[+] metrics_logger.py çıktıları alındı.")
            except subprocess.TimeoutExpired:
                print("[!] metrics_logger.py çıktısı alınamadı (communicate timeout). Zorla sonlandırılıyor.")
                if metrics_proc.poll() is None: # Hala çalışıyorsa
                    os.killpg(os.getpgid(metrics_proc.pid), signal.SIGTERM)
                    metrics_proc.wait(timeout=5)
            except Exception as e:
                print(f"[!] metrics_logger.py çıktısı işlenirken hata: {e}")
                if metrics_proc.poll() is None: # Hata oldu ama hala çalışıyorsa sonlandır
                    os.killpg(os.getpgid(metrics_proc.pid), signal.SIGTERM)
                    metrics_proc.wait(timeout=5)
        elif metrics_proc:
            print("[+] metrics_logger.py zaten sonlanmıştı.")
        else:
            print("[!] metrics_logger.py process objesi hiç oluşturulmadı.")

        # logger.py'nin tüm çıktısını result_txt dosyasına ekle (hem stdout hem stderr)
        with open(result_txt_file, "a") as f:
            f.write("\n--- metrics_logger.py Output ---\n")
            f.write(metrics_outs.decode("utf-8", errors='ignore'))
            if metrics_errs:
                f.write("\n--- metrics_logger.py Errors ---\n")
                f.write(metrics_errs.decode("utf-8", errors='ignore'))

        # logger.py'nin yazdığı metrik CSV dosyasını taşı (eğer varsa)
        source_metrics_dir = os.path.join(EXPERIMENTS_DIR, "metrics") # logger'ın varsayılan output dizini
        source_metrics_path_temp = os.path.join(source_metrics_dir, f"{algo_name}.csv") # logger'ın yazdığı dosya

        # Dosyanın yazılması için kısa bir bekleme (communicate timeout'a düştüyse faydalı olabilir)
        time.sleep(1)

        if os.path.exists(source_metrics_path_temp):
            try:
                shutil.move(source_metrics_path_temp, metrics_csv_file)
                print(f"[✓] Metrikler kaydedildi: {metrics_csv_file}")
                # Şimdi bu CSV dosyasından veriyi çekip ana summary.csv'ye ekle
                nav_metric_data = None
                with open(metrics_csv_file, 'r') as f:
                    reader = csv.reader(f)
                    header = next(reader, None) # Başlığı atla
                    nav_metric_data = next(reader, None) # İlk veri satırını al (tek satır bekleniyor)

                if nav_metric_data:
                    add_navigation_metric_to_csv(nav_metric_data)
                    print(f"[+] Metrikler özet CSV'ye eklendi: {nav_metric_data[0:3]}...") # İlk 3 sütunu göster
                else:
                    print(f"[!] Metrik CSV dosyasında veri bulunamadı veya boş: {metrics_csv_file}")
            except Exception as e:
                print(f"[!] Metrik CSV dosyası işlenirken hata oluştu: {e}")
        else:
            print(f"[!] Metrik dosyası bulunamadı: {source_metrics_path_temp}. Otomatik taşıma yapılamadı.")


# --- Ana Çalışma Bloğu ---

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Barn Challenge Otomatik Test Koşucusu")
    parser.add_argument('--start_world', type=int, required=True, help="Başlangıç world indeksi (dahil)")
    parser.add_argument('--end_world', type=int, required=True, help="Bitiş world indeksi (dahil)")
    parser.add_argument('--num_trials', type=int, default=1, help="Her world için deneme(trial) sayısı")
    parser.add_argument('--algorithms', nargs='*', default=list(ALGORITHMS.keys()),
                        help=f"Koşulacak algoritmalar (hepsi için boş bırak). Mevcutlar: {', '.join(ALGORITHMS.keys())}")
    args = parser.parse_args()

    roscore_process = None
    if not is_roscore_running():
        roscore_process = start_roscore()
        if roscore_process is None:
            print("❌ roscore başlatılamadığı için program sonlandırılıyor.")
            exit(1)
        time.sleep(2) # roscore'un tam olarak başlaması için bekle

    create_summary_csv() # Özet CSV dosyasını oluştur

    for algo in args.algorithms:
        if algo not in ALGORITHMS:
            print(f"[-] Bilinmeyen algoritma: {algo}. Atlanıyor.")
            continue

        config = ALGORITHMS[algo]
        for world_idx in range(args.start_world, args.end_world + 1):
            for trial in range(1, args.num_trials + 1):
                run_single_experiment(algo, config, world_idx, trial)

    print("\n✅ Tüm algoritmalar test edildi. Sonuçlar summary_results.csv ve logs klasöründe!")

    # roscore'u sonlandır (eğer bu script başlattıysa)
    if roscore_process is not None:
        print("[+] roscore sonlandırılıyor...")
        try:
            os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM)
            roscore_process.wait(timeout=5)
            print("[✓] roscore başarıyla sonlandırıldı.")
        except Exception as e:
            print(f"[!] roscore sonlandırılamadı: {e}")
