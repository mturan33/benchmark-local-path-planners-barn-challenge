#!/bin/bash
source ~/venvs/barn_e2e/bin/activate
cd ~/3v3/ws_barn_e2e/src/the-barn-challenge
source /opt/ros/melodic/setup.bash
source ~/3v3/ws_barn_e2e/devel/setup.bash
python3 run.py --test --out_path ~/3v3/experiments/results/e2e_out.txt
