#!/usr/bin/env python3

import os
import subprocess
import random
import time
import shutil

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

def run_with_metrics(algo, config, world_idx, trial=1):
    print(f"\n=== {algo.upper()} | World: {world_idx} | Trial: {trial} ===")

    result_txt = os.path.join(RESULTS_PATH, f"{algo}_out_world_{world_idx}_trial_{trial}.txt")
    result_csv = os.path.join(RESULTS_PATH, f"metrics_{algo}_world_{world_idx}_trial_{trial}.csv")
    metrics_log_file = os.path.join(METRICS_PATH, f"metrics_log_{algo}.csv")

    # 1. Metrics logger'ı başlat (cwd = ~/3v3/experiments)
    metrics_cmd = (
        "source /opt/ros/melodic/setup.bash && "
        f"python3 metrics_logger.py _algorithm_name:={algo}"
    )
    metrics_proc = subprocess.Popen(
        ["/bin/bash", "-c", metrics_cmd],
        cwd=EXPERIMENTS_PATH,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setpgrp
    )
    time.sleep(5)  # logger subscribe olabilsin

    # 2. Algoritmayı başlat
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
            result = subprocess.run(
                ["/bin/bash", "-c", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                cwd=work_dir,
                timeout=300
            )
            output = result.stdout.decode("utf-8")
            outfile.write(output)
        except subprocess.TimeoutExpired:
            outfile.write("⏱ TIMEOUT\n")
            print(f"[!] {algo.upper()} TIMEOUT!")

    # 3. metrics_logger.py'nin çıktısını göster (debug için)
    try:
        outs, errs = metrics_proc.communicate(timeout=10)
        print(f"[metrics_logger.py stdout]\n{outs.decode()}")
        print(f"[metrics_logger.py stderr]\n{errs.decode()}")
    except Exception:
        print("[metrics_logger.py] Hızlı kapanma veya uzun süreli çalışıyor olabilir.")

    # 4. Metrics dosyasını taşı
    if os.path.exists(metrics_log_file):
        shutil.move(metrics_log_file, result_csv)
        print(f"[✓] Metrics saved: {result_csv}")
    else:
        print(f"[!] Metrics file not found for {algo}.")

if __name__ == "__main__":
    NUM_TRIALS = 1
    for algo, config in ALGORITHMS.items():
        world_idx = random.randint(0, 299)
        for trial in range(1, NUM_TRIALS + 1):
            run_with_metrics(algo, config, world_idx, trial)

    print("\n✅ Tüm algoritmalar test edildi. Sonuçlar ~/3v3/experiments/results/ klasöründe!\n")

