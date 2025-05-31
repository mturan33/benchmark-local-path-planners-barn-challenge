# ~/3v3/experiments/report_all.py
import os
import subprocess

results_dir = os.path.expanduser("~/3v3/experiments/results")
report_script = os.path.expanduser("~/3v3/ws_barn_dwa_eband_teb/src/the-barn-challenge/report_test.py")

print("🚀 Toplu Rapor Başlatılıyor:\n")
for file in sorted(os.listdir(results_dir)):
    if file.endswith(".txt"):
        out_path = os.path.join(results_dir, file)
        print(f"📄 {file}")
        subprocess.run(["python3", report_script, "--out_path", out_path])
        print("-" * 60)

