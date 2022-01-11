#!/usr/bin/zsh


# ros2 param list [<node_name>]

source ./install/setup.zsh          \
 && echo '/parameters has params:'  \
 && ros2 param list /parameters
