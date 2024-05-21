# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "gazebo_msgs;geometry_msgs;message_generation;message_runtime;nav_msgs;rigid2d;roscpp;sensor_msgs;std_msgs;visualization_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lnuslam;-lrigid2d".split(';') if "-lnuslam;-lrigid2d" != "" else []
PROJECT_NAME = "nuslam"
PROJECT_SPACE_DIR = "/home/vivek/catkin_ws/install"
PROJECT_VERSION = "0.0.0"
