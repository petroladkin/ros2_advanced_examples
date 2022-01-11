#!/usr/bin/zsh


# ros2 param set <node_name> <parameter_name> <value>

source ./install/setup.zsh                              \
 && echo 'set debug_logger parameter:'                  \
 && ros2 param set /parameters debug_logger True        \
 && echo 'set read_on_start_param parameter:'           \
 && ros2 param set /parameters read_on_start_param 100  \
 && echo 'set monitoring_param parameter:'              \
 && ros2 param set /parameters monitoring_param "hello ROS2"
