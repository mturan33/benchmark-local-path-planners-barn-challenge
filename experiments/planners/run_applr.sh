#!/bin/bash
source ~/venvs/barn_applr/bin/activate
cd ~/3v3/ws_barn_applr/src/the-barn-challenge
source /opt/ros/melodic/setup.bash
source ~/3v3/ws_barn_applr/devel/setup.bash
python3 run.py --test --out_path ~/3v3/experiments/results/applr_out.txt
