# Benchmark of Local Path Planners in ROS using the BARN Dataset

![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![ROS Version](https://img.shields.io/badge/ROS-Melodic-blue.svg)
![Python Version](https://img.shields.io/badge/Python-3.6+-brightgreen.svg)

![Comparative Results Plot](https://github.com/user-attachments/assets/22d543e7-b657-4ca0-bd57-44f9ff2dc5ee)

## Overview

This repository provides a comprehensive and reproducible framework for benchmarking local path planning algorithms within the ROS ecosystem. The primary goal is to systematically evaluate and compare the performance of classical and learning-based planners using the challenging **BARN (Benchmark for Agile Robot Navigation) dataset**.

Autonomous navigation in complex environments is a critical challenge in robotics. While numerous local planners exist, the literature often lacks standardized, quantitative comparisons that can guide researchers and practitioners. This framework addresses that gap by providing a unified testbed, automated logging tools, and analysis scripts to compare planners across a wide range of metrics.

A research paper detailing the findings from this framework is currently under review and will be published soon.

## Key Features

-   **Reproducible Framework:** Standardized ROS/Gazebo setup for testing seven different local planners under identical conditions across 300 unique maps.
-   **Automated Metric Collection:** Custom ROS logger nodes that automatically capture over 14 key performance metrics, including success rate, total time, path efficiency, control effort, and path smoothness.
-   **Modular Workspace Structure:** Each learning-based planner is isolated in its own Catkin workspace and Python virtual environment to manage complex and often conflicting dependencies.
-   **Analysis and Visualization Scripts:** Includes Python scripts to process raw metric data, perform statistical significance tests (Wilcoxon signed-rank test with Holm-Bonferroni correction), and generate comparative plots and heatmaps.

## Algorithms Benchmarked

This study evaluates seven distinct planners, grouped into two categories:

#### Classical Planners
-   **DWA** (Dynamic Window Approach)
-   **TEB** (Timed Elastic Band)
-   **EBand** (Elastic Band)
-   **FastDWA** (A performance-optimized variant of DWA)

#### Learning-Based Planners
-   **SAC** (Soft Actor-Critic)
-   **APPLR** (Adaptive Planner Parameter Learning from Reinforcement)
-   **LfH** (Learning from Hallucination)

![Gazebo Simulation Environment](https://github.com/user-attachments/assets/52121c44-d66f-4e9b-b239-3c58913ea7f4)

## Repository Structure

The repository is organized to ensure modularity and ease of use. Each learning-based planner has a dedicated ROS workspace and Python environment to avoid dependency conflicts.

```
/
├── ws_barn_dwa_eband_teb/     # ROS workspace for core classical planners
├── ws_barn_fastdwa/           # ROS workspace for FastDWA
├── ws_barn_applr/             # ROS workspace for APPLR
├── ws_barn_lfh/               # ROS workspace for LfH
├── ws_barn_sac/               # ROS workspace for SAC
│
├── experiments/               # Main directory for running tests and analysis
│   ├── metrics/               # Raw and summarized metric CSV files are stored here
│   ├── planners/              # Launch and config files for each planner
│   ├── results/               # Output directory for generated graphs and plots
│   ├── run_experiment.py      # Master script to run experiments in batches
│   ├── metric_statistics.py   # Script for statistical analysis and heatmap generation
│   └── ...
│
└── venvs/                     # Python virtual environments for each learning-based planner
```

## Installation and Setup

This framework is designed for **Ubuntu 18.04 with ROS Melodic**.

#### 1. Prerequisites
-   Ubuntu 18.04
-   ROS Melodic Morenia (Desktop-Full Install recommended)
-   Python 3.6+ and `pip`

#### 2. Clone the Repository
```bash
git clone https://github.com/mturan33/benchmark-local-path-planners-barn-challenge.git 3v3
cd 3v3
```

#### 3. Setup Python Virtual Environments
Each learning-based planner requires a specific set of Python packages. We use virtual environments to keep them isolated.

```bash
# Create and activate an environment (example for SAC)
python3 -m venv venvs/sac_env
source venvs/sac_env/bin/activate

# Install required packages from its requirements file
# Note: You will need to create a requirements.txt file for each environment based on the dependencies.
pip install -r ws_barn_sac/requirements.txt 
# Example requirements might include: torch, gym==0.21.0, numpy, rospkg, etc.

# Deactivate when done
deactivate
```
*Repeat this process for each learning-based planner (LfH, Applr) using their respective requirements.*

**Important Note on ROS & Python:** ROS Melodic uses Python 2 by default, but many modern RL libraries require Python 3. The provided scripts and workspaces are configured to handle this, but ensure you install system-level ROS dependencies correctly. For packages like `pyyaml` needed by ROS tools, you may need to install it for Python 2:
```bash
sudo apt-get install python-yaml
# or if using pip:
# sudo python2 -m pip install pyyaml
```

#### 4. Build ROS Workspaces
You need to build each Catkin workspace individually.

```bash
# Example for the classical planners workspace
source /opt/ros/melodic/setup.bash
cd ws_barn_dwa_eband_teb/
catkin_make
cd ..
```
*Repeat the `catkin_make` process for every `ws_*` directory.*

## Running Experiments

All experiments are managed from the `experiments/` directory.

#### 1. Running a Single Trial
To run a single experiment for a specific planner on a specific world:

```bash
# 1. Navigate to the experiments directory
cd experiments/

# 2. Activate the correct Python environment if needed (for learning-based planners)
# source ../venvs/sac_env/bin/activate

# 3. Source the relevant ROS workspace
source ../ws_barn_dwa_eband_teb/devel/setup.bash

# 4. Execute the master run script
# Usage: python3 run_experiment.py --planner <planner_name> --world <world_id> [--gui]
python3 run_experiment.py --planner dwa --world 10 --gui
```
-   `<planner_name>` can be `dwa`, `teb`, `eband`, `fastdwa`, `sac`, `applr`, `lfh`.
-   `<world_id>` is a number from 0 to 299.
-   `--gui` is an optional flag to launch the Gazebo client for visualization.

#### 2. Running in Batches
The `run_experiment.py` script can be easily modified or wrapped in a bash script to run trials in a loop. For example:```bash
#!/bin/bash
PLANNERS=("dwa" "teb" "eband" "sac")
for planner in "${PLANNERS[@]}"; do
  for i in {0..299}; do
    echo "Running $planner on world $i"
    python3 run_experiment.py --planner "$planner" --world "$i"
  done
done```

The script automatically handles launching the correct ROS nodes, Gazebo world, and the metric logger for each trial.

## Analyzing Results

After running the experiments, raw data is saved in `experiments/metrics/`. You can generate summary statistics and publication-ready plots using the provided analysis script.

```bash
# Navigate to the experiments directory
cd experiments/

# Run the statistical analysis script
python3 metric_statistics.py
```
This script will:
1.  Load and clean the data from all algorithm runs.
2.  Print summary tables with mean values and 95% confidence intervals.
3.  Generate and save statistical significance heatmaps (using Wilcoxon test + Holm correction) in the `statistical_results/` directory.

## How to Cite

If you use this framework, code, or results in your research, please consider citing our upcoming paper (placeholder below) and the original BARN dataset paper.

**Our Paper (Placeholder) Coming Soon:**
```bibtex
@article{YardimciCogurcu2025,
  title={Benchmarking Local Path Planners in ROS using the BARN Dataset},
  author={Yardimci, Mehmet Turan and Çogurcu, Yunus Emre},
  journal={Çukurova University Journal of the Faculty of Engineering},
  year={0},
  volume={0},
  number={0}
}
```

**BARN Dataset Paper:**
```bibtex
@inproceedings{xiao2021agile,
  title={Agile Robot Navigation through Hallucinated Learning and Sober Deployment},
  author={Xiao, Xiaojian and Liu, Bo and Stone, Peter},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={11501--11507},
  year={2021},
  organization={IEEE}
}
```

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

For questions, contributions, or collaborations, please open an issue or contact [mehmetturan2003@gmail.com](mailto:mehmetturan2003@gmail.com).
