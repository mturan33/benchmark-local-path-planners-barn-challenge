#!/usr/bin/env python3

import os
import subprocess
import time
import argparse

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

    # 1. Metrics logger'ı başlat (ENV değişkenleri ile)
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

    print(f"[✓] Metrics (summary) dosyası güncellendi: {METRICS_PATH}/metrics_summary_{algo}.csv")

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

