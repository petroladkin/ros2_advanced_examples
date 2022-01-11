#!/usr/bin/zsh


# ros2 param get <node_name> <parameter_name>

source ./install/setup.zsh                              \
 && echo 'get debug_logger parameter:'                  \
 && ros2 param get /parameters debug_logger             \
 && echo 'get read_on_start_param parameter:'           \
 && ros2 param get /parameters read_on_start_param      \
 && echo 'get monitoring_param parameter:'              \
 && ros2 param get /parameters monitoring_param
