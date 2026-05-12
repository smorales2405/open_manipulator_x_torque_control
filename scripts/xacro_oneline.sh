#!/usr/bin/env bash
# Produces a single-line, comment-free URDF string.
# Required because gazebo_ros2_control 0.4.x passes robot_description as
# --param robot_description:=<xml> to rclcpp::NodeOptions.  rcl uses
# yaml-cpp to parse the value, which fails on two things:
#   1. Newlines  → tr -d '\n'
#   2. ': ' (colon-space) inside XML comments → strip all <!-- ... -->
# XML comments are not consumed by any ros2_control component, so removing
# them is safe.
xacro "$@" | python3 -c "
import sys, re
content = sys.stdin.read()
content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)
content = content.replace('\n', '')
sys.stdout.write(content)
"
