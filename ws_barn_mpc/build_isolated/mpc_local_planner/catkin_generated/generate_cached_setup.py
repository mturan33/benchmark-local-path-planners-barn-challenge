# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/turan/3v3/ws_barn_mpc/devel_isolated/mpc_local_planner_msgs;/home/turan/3v3/ws_barn_mpc/devel_isolated/mpc_local_planner_examples;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_viz;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_tutorials;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_simulator;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_navigation;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_msgs;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_helper;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_gazebo;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_desktop;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_description;/home/turan/3v3/ws_barn_mpc/devel_isolated/jackal_control;/home/turan/3v3/ws_barn_mpc/devel_isolated/eband_local_planner;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/turan/3v3/ws_barn_mpc/devel_isolated/mpc_local_planner/env.sh')

output_filename = '/home/turan/3v3/ws_barn_mpc/build_isolated/mpc_local_planner/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
