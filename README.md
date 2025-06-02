
# Benchmark of Local Path Planners in the BARN Challenge Dataset

This repository provides a complete experimental framework for benchmarking and comparing various classical and learning-based local path planning algorithms on the BARN Challenge simulation environments using 2D LiDAR point clouds.

## Features

	Algorithm	World	Trial	Total_Time	Path_Length	Deviation_From_Straight	Avg_Velocity	Path_Efficiency	Collision
	dwa		    0	    1	    93.69		19.3		1.93			        0.21		    0.518		    1
	dwa		    0	    1	    28.52		6.17		0.617			        0.22		    1.62		    1
	dwa		    0	    1	    52.69		6.65		0.665			        0.13		    1.504		    1
	dwa		    0	    1	    31.57		10.51		1.051			        0.33		    0.951		    0

- Reproducible experiment scripts for classical planners (DWA, EBand, TEB, etc.) and modern RL-based planners (SAC, LfH, E2E, etc.)
- Unified logging and metrics collection (path length, arrival time, collision, efficiency, etc.)
- Ready-to-use Python scripts for experiment management, batch runs, and results analysis
- Clean workspace structure with separate ROS workspaces for nearly each planner
- .gitignore and best-practices for multi-algorithm ROS/Catkin repositories

## Directory Structure

```
~/3v3/
├── ws_barn_dwa_eband_teb/
├── ws_barn_fastdwa/
├── ws_barn_applr/
├── ws_barn_e2e/
├── ws_barn_lfh/
├── ws_barn_sac/
├── experiments/
│   ├── metrics/
│   ├── planners/
│   ├── results/
│   └── run_experiments.py
└── venvs/
```

## Quick Start

1. **Clone the repository:**
    ```bash
    git clone https://github.com/mturan33/benchmark-local-path-planners-barn-challenge.git 3v3
    ```

2. **Install dependencies:**  
   See each workspace's README or the scripts inside the `experiments/` folder for requirements and instructions.
    pip install gym==0.21.0
    pip install rospkg
    pip install rospkg catkin_pkg PyYAML empy defusedxml
    pip install PyYAML
    pip install --upgrade pip
    pip install numpy scipy matplotlib defusedxml pyyaml netifaces
    pip install numpy
    pip install scipy torch
    pip install defusedxml
    pip install gym
    pip install gym==0.26.2
    pip install --upgrade pip setuptools wheel
    pip install rospkg catkin_pkg PyYAML empy
    pip install pyyaml
    sudo python2 -m pip install pyyaml
    pip install numpy==1.21.6 matplotlib==3.5.3 scipy rospkg pyyaml defusedxml netifaces torch==1.7 torchvision==0.8 tensorboard --only-binary=:all:

4. **Run experiments:**  
   Example for DWA planner:
    ```bash
    source ~/3v3/venvs/barn_dwa/bin/activate
    cd ~/3v3/ws_barn_dwa_eband_teb/src/the-barn-challenge
    source /opt/ros/melodic/setup.bash
    source ~/3v3/ws_barn_dwa_eband_teb/devel/setup.bash
    python3 run_dwa.py --world_idx 0 --gui
    ```

5. **Collect metrics:**  
   The logger scripts will automatically generate and append summary CSV files in `experiments/metrics/`.
   Example for DWA planner Logger:
    ```bash
    cd ~/3v3/experiments
    WORLD_IDX=0 TRIAL=1 python3 metrics_logger_summary.py _algortihm_name:=dwa
    ```

## Citation and License

If you use this repository or scripts for your research, please cite the original [BARN Challenge](https://cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge25.html) and related papers.

MIT License © Mehmet Turan Yardımcı

---

For questions, contributions, or collaborations, feel free to open an issue or contact [mehmetturan2003@gmail.com](mailto:mehmetturan2003@gmail.com).
