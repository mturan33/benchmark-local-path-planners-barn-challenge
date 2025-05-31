#!/bin/bash
source ~/venvs/barn_lfh/bin/activate
cd ~/3v3/ws_barn_lfh/src/the-barn-challenge
source /opt/ros/melodic/setup.bash
source ~/3v3/ws_barn_lfh/devel/setup.bash
python3 run.py --test --out_path ~/3v3/experiments/results/lfh_out.txt
