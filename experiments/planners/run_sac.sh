#!/bin/bash
source ~/venvs/barn_sac/bin/activate
cd ~/3v3/ws_barn_sac/src/the-barn-challenge
source /opt/ros/melodic/setup.bash
source ~/3v3/ws_barn_sac/devel/setup.bash
python3 run.py --test --out_path ~/3v3/experiments/results/sac_out.txt
