#!/usr/bin/zsh


# ros2 action send_goal --feedback <action_name> <action_type> <values>

source ./install/setup.zsh\
 && ros2 action send_goal --feedback /action/wait interfaces/action/Wait "{count: 10, timeout: 1.0}"
