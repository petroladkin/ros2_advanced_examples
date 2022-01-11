#!/usr/bin/zsh


# ros2 param dump [--output-dir <dumps_dir>] <node_name>

source ./install/setup.zsh                                \
 && mkdir -p ./dumps                                      \
 && echo 'dump parameters:'                               \
 && ros2 param dump --output-dir ./dumps /parameters
