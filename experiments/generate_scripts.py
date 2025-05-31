# ~/3v3/experiments/generate_scripts.py
import os

algos = {
    "dwa":      ("ws_barn_dwa_eband_teb", "run_dwa.py"),
    "eband":    ("ws_barn_dwa_eband_teb", "run_eband.py"),
    "teb":      ("ws_barn_dwa_eband_teb", "run_teb.py"),
    "sac":      ("ws_barn_sac", "run.py"),
    "lfh":      ("ws_barn_lfh", "run.py"),
    "applr":    ("ws_barn_applr", "run.py"),
    "e2e":      ("ws_barn_e2e", "run.py")	
}

script_template = """#!/bin/bash
source ~/venvs/barn_{algo}/bin/activate
cd ~/3v3/{ws}/src/the-barn-challenge
source /opt/ros/melodic/setup.bash
source ~/3v3/{ws}/devel/setup.bash
python3 {run_script} --test --out_path ~/3v3/experiments/results/{algo}_out.txt
"""

out_dir = os.path.expanduser("~/3v3/experiments/planners")
os.makedirs(out_dir, exist_ok=True)

for algo, (ws, run_script) in algos.items():
    with open(f"{out_dir}/run_{algo}.sh", "w") as f:
        f.write(script_template.format(algo=algo, ws=ws, run_script=run_script))
    os.chmod(f"{out_dir}/run_{algo}.sh", 0o755)

print("✅ run_*.sh dosyaları oluşturuldu.")

