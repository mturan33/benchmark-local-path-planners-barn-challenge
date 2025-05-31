#!/bin/bash
source ~/3v3/venvs/barn_fastdwa/bin/activate
cd ~/3v3/ws_barn_fastdwa/src/the-barn-challenge
source /opt/ros/melodic/setup.bash
source ~/3v3/ws_barn_fastdwa/devel/setup.bash
python3 run.py --test --out_path ~/3v3/experiments/results/dwa_out.txt
