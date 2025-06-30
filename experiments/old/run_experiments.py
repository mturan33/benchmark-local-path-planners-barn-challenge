#!/usr/bin/env python3

import os
import subprocess
import random
import time

ALGORITHMS = {
    "dwa": {
        "venv": "barn_dwa",
        "ws": "ws_barn_dwa_eband_teb",
        "script": "run_dwa.py"
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
METRICS_LOGGER = os.path.join(BASE_PATH, "experiments/metrics_logger.py")
os.makedirs(RESULTS_PATH, exist_ok=True)

# Rastgele bir dünya seç
world_idx = random.randint(0, 299)
print(f"\n🌍 Selected world_idx: {world_idx}\n")

def run_test(algo, config):
    print(f"🧪 Running {algo.upper()} on world {world_idx}...")

    # Script ve çalışma dizini
    ws_path = os.path.join(BASE_PATH, config["ws"])
    script_dir = os.path.join(ws_path, "src", "the-barn-challenge")
    script_path = os.path.join(script_dir, config["script"])
    result_file = os.path.join(RESULTS_PATH, f"{algo}_out_world_{world_idx}.txt")

    cmd = f"""
    source ~/3v3/venvs/{config['venv']}/bin/activate &&
    cd {script_dir} &&
    source /opt/ros/melodic/setup.bash &&
    source {ws_path}/devel/setup.bash &&
    python3 {METRICS_LOGGER} _algorithm_name:={algo} &
    MET_PID=$! &&
    sleep 3 &&
    python3 {config['script']} --world_idx {world_idx} &&
    sleep 1 &&
    kill $MET_PID
    """

    with open(result_file, "w") as outfile:
        try:
            result = subprocess.run(["/bin/bash", "-c", cmd],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT,
                                    timeout=60)
            output = result.stdout.decode("utf-8")
            outfile.write(output)
        except subprocess.TimeoutExpired:
            outfile.write("⏱ TIMEOUT\n")

if __name__ == "__main__":
    for algo, config in ALGORITHMS.items():
        run_test(algo, config)
    print(f"\n✅ All planners tested on world {world_idx}. Results saved to ~/3v3/experiments/results/\n")

