#!/usr/bin/zsh


# ros2 param load <node_name> <parameter_file>

source ./install/setup.zsh                                \
 && echo 'load parameters from ./dumps/parameters.yaml:'  \
 && ros2 param load /parameters ./dumps/parameters.yaml
