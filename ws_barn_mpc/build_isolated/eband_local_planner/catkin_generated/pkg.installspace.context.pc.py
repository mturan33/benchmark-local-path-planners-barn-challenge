# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "base_local_planner;control_toolbox;costmap_2d;geometry_msgs;nav_core;nav_msgs;pluginlib;roscpp;tf2_eigen;tf2_geometry_msgs;tf2_ros;dynamic_reconfigure".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-leband_local_planner".split(';') if "-leband_local_planner" != "" else []
PROJECT_NAME = "eband_local_planner"
PROJECT_SPACE_DIR = "/home/turan/3v3/ws_barn_mpc/install_isolated"
PROJECT_VERSION = "0.4.0"
