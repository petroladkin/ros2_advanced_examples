#!/usr/bin/zsh


# ros2 action list -t

source ./install/setup.zsh\
 && echo 'actions:'\
 && ros2 action list -t
