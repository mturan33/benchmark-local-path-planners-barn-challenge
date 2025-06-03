#!/usr/bin/env python3

import os
import subprocess
import time
import argparse
import signal
import socket
import csv
import re

def is_roscore_running():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.settimeout(1)
        s.connect(("localhost", 11311))
        s.close()
        return True
    except Exception:
        return False

def start_roscore():
    print("[run_all_with_metrics] roscore çalışmıyor, başlatılıyor...")
    proc = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid
    )
    for _ in range(10):
        if is_roscore_running():
            print("[run_all_with_metrics] roscore başarıyla başlatıldı!")
            return proc
        time.sleep(1)
    print("[run_all_with_metrics] roscore başlatılamadı, elle kontrol edin!")
    return None

roscore_proc = None
if not is_roscore_running():
    roscore_proc = start_roscore()
    time.sleep(2)
else:
    print("[run_all_with_metrics] roscore zaten çalışıyor.")

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
    }
}

BASE_PATH = os.path.expanduser("~/3v3")
RESULTS_PATH = os.path.join(BASE_PATH, "experiments/results")
METRICS_PATH = os.path.join(BASE_PATH, "experiments/metrics")
EXPERIMENTS_PATH = os.path.join(BASE_PATH, "experiments")
os.makedirs(RESULTS_PATH, exist_ok=True)
os.makedirs(METRICS_PATH, exist_ok=True)

def extract_navigation_metric(result_txt_path):
    try:
        with open(result_txt_path, 'r') as f:
            for line in f:
                match = re.search(r'Navigation metric:\s*([0-9\.\-eE]+)', line)
                if match:
                    return float(match.group(1))
    except Exception as e:
        print(f"[WARN] {result_txt_path} okunamadı: {e}")
    return -1

def add_navigation_metric_to_csv(csv_path, nav_metric):
    with open(csv_path, 'r') as f:
        reader = list(csv.reader(f))
        header = reader[0]
        rows = reader[1:]

    if "Navigation_Metric" not in header:
        header.append("Navigation_Metric")

    if rows:
        # Eğer son satırda zaten navigation_metric varsa, güncelle
        if len(rows[-1]) == len(header) - 1:
            rows[-1].append(str(nav_metric))
        else:
            rows[-1][len(header) - 1] = str(nav_metric)

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)

def wait_for_file(filepath, timeout=5.0, poll=0.1):
    t0 = time.time()
    while not os.path.exists(filepath):
        time.sleep(poll)
        if time.time() - t0 > timeout:
            return False
    return True

def run_with_metrics(algo, config, world_idx, trial=1):
    summary_csv = os.path.join(METRICS_PATH, f"metrics_summary_{algo}.csv")

    print(f"\n=== {algo.upper()} | World: {world_idx} | Trial: {trial} ===")

    result_txt = os.path.join(RESULTS_PATH, f"{algo}_out_world_{world_idx}_trial_{trial}.txt")

    metrics_env = os.environ.copy()
    metrics_env["WORLD_IDX"] = str(world_idx)
    metrics_env["TRIAL"] = str(trial)
    metrics_cmd = (
        "source /opt/ros/melodic/setup.bash && "
        f"python3 metrics_logger.py _algorithm_name:={algo}"
    )
    metrics_proc = subprocess.Popen(
        ["/bin/bash", "-c", metrics_cmd],
        cwd=EXPERIMENTS_PATH,
        env=metrics_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setpgrp
    )
    time.sleep(5)

    script_path = os.path.join(BASE_PATH, config["ws"], "src", "the-barn-challenge", config["script"])
    work_dir = os.path.dirname(script_path)
    cmd = (
        f"source ~/3v3/venvs/{config['venv']}/bin/activate && "
        f"cd {work_dir} && "
        "source /opt/ros/melodic/setup.bash && "
        f"source {BASE_PATH}/{config['ws']}/devel/setup.bash && "
        f"python3 {config['script']} --world_idx {world_idx}"
    )
    with open(result_txt, "w") as outfile:
        try:
            proc = subprocess.Popen(
                ["/bin/bash", "-c", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                cwd=work_dir,
                preexec_fn=os.setsid
            )
            found_done = False
            while True:
                line = proc.stdout.readline()
                if not line:
                    break
                decoded = line.decode("utf-8")
                outfile.write(decoded)
                outfile.flush()
                if "Test finished" in decoded or "Navigation timeout" in decoded or "done" in decoded.lower():
                    found_done = True
                    print(f"[{algo.upper()}] Tamamlandı, tüm process grubunu kapatıyorum.")
                    time.sleep(5)
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)

                    # Sonuç dosyası oluştu mu?
                    if not os.path.exists(summary_csv):
                        print(f"[ERROR] {summary_csv} dosyası oluşmadı!")
                    else:
                        print(f"[✓] Metrics (summary) dosyası güncellendi: {summary_csv}")

                    break
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print(f"[!] {algo.upper()} TIMEOUT! Tüm processleri sonlandırıyorum.")
            time.sleep(5)
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception as e:
            print(f"[{algo.upper()}] Hata: {e}")

    try:
        outs, errs = metrics_proc.communicate(timeout=10)
        print(f"[metrics_logger.py stdout]\n{outs.decode()}")
        print(f"[metrics_logger.py stderr]\n{errs.decode()}")
    except Exception:
        print("[metrics_logger.py] Hızlı kapanma veya uzun süreli çalışıyor olabilir.")

    print(f"[✓] Metrics (summary) dosyası güncellendi: {METRICS_PATH}/metrics_summary_{algo}.csv")

    summary_csv = os.path.join(METRICS_PATH, f"metrics_summary_{algo}.csv")
    if not wait_for_file(summary_csv, timeout=7):
        print(f"[ERROR] {summary_csv} dosyası oluşmadı, navigation_metric eklenemiyor!")
    else:
        nav_metric = extract_navigation_metric(result_txt)
        add_navigation_metric_to_csv(summary_csv, nav_metric)
        print(f"[+] Navigation metric ({nav_metric}) CSV'ye eklendi!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Barn Challenge Otomatik Koşucu")
    parser.add_argument('--start_world', type=int, required=True, help="Başlangıç world indeksi (dahil)")
    parser.add_argument('--end_world', type=int, required=True, help="Bitiş world indeksi (dahil)")
    parser.add_argument('--num_trials', type=int, default=1, help="Her world için deneme(trial) sayısı")
    parser.add_argument('--algorithms', nargs='*', default=list(ALGORITHMS.keys()), help="Koşulacak algoritmalar (hepsi için boş bırak)")
    args = parser.parse_args()

    for algo in args.algorithms:
        config = ALGORITHMS[algo]
        for world_idx in range(args.start_world, args.end_world + 1):
            for trial in range(1, args.num_trials + 1):
                run_with_metrics(algo, config, world_idx, trial)

    print("\n✅ Tüm algoritmalar test edildi. Sonuçlar summary csv ve loglar klasöründe!\n")

    if roscore_proc is not None:
        print("[run_all_with_metrics] roscore sonlandırılıyor...")
        try:
            time.sleep(5)
            os.killpg(os.getpgid(roscore_proc.pid), signal.SIGTERM)
            print("[run_all_with_metrics] roscore kapatıldı.")
        except Exception as e:
            print(f"[run_all_with_metrics] roscore kapatılamadı: {e}")

