# AUAV 2022 Sample

This is a demonstration of how to use PX4 avoidance on Ubuntu 20.04 and ROS noetic for the IEEE Autonomous UAS 2022 competition.

source /opt/ros/noetic/setup.bash

# Source git/catkin_ws directory
# source ~/git/catkin_ws/devel/setup.bash
# source ~/git/forked/devel/setup.bash
source ~/git/forked2/devel/setup.bash

# Trial Gazebo Simulations path updates
export GAZEBO_PLUGIN_PATH=/home/scoops12/git/uavcc-simulator/trial_1_setup:/home/scoops12/git/catkin_ws/install/lib/mavlink_sitl_gazebo/plugins:/home/scoops12/git/uavcc-simulator/trial_2_setup:
export GAZEBO_MODEL_PATH=/home/scoops12/git/catkin_ws/install/share/mavlink_sitl_gazebo/models:/home/scoops12/git/uavcc-simulator/trial_1_setup:/home/scoops12/git/uavcc-simulator/trial_2_setup:/home/scoops12/.gazebo/models/trial_1_setup:/home/scoops12/.gazebo/models/trial_2_setup:
