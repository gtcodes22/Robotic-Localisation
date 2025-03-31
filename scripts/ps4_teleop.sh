#!/bin/bash

ros2 run teleop_twist_joy teleop_node \
  --ros-args \
  -p enable_button:=6 \
  -p enable_turbo_button:=7 \
  -p axis_linear.x:=1 \
  -p axis_angular.yaw:=3 \
  -p scale_linear.x:=0.5 \
  -p scale_angular.yaw:=0.8 \
  -p scale_linear_turbo.x:=1.2 \
  -p scale_angular_turbo.yaw:=1.5 \
  -p axis_deadzone:=0.05 \
  -p require_enable_button:=True
