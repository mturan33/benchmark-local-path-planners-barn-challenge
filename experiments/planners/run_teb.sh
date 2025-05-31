#!/bin/bash
source ~/venvs/barn_teb/bin/activate
cd ~/3v3/ws_barn_dwa_eband_teb/src/the-barn-challenge
source /opt/ros/melodic/setup.bash
source ~/3v3/ws_barn_dwa_eband_teb/devel/setup.bash
python3 run_teb.py --test --out_path ~/3v3/experiments/results/teb_out.txt
