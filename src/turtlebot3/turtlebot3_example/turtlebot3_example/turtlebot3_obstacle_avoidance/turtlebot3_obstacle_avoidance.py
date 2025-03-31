#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Enhanced Avoidance Enabled')
        print('----------------------------------------------')
        print('Slowdown distance: 0.8 m')
        print('Stop distance: 0.4 m')
        print('Clearance distance: 0.6 m')
        print('----------------------------------------------')

        self.scan_ranges = []
        self.has_scan_received = False

        # Enhanced parameters
        self.slowdown_distance = 0.8  # Start slowing down at this distance
        self.stop_distance = 0.4      # Stop completely at this distance
        self.clearance_distance = 0.6 # Resume moving when path is this clear
        self.emergency_distance = 0.3 # Immediate stop distance
        
        # Behavior parameters
        self.max_speed = 0.26         # Maximum linear speed (m/s)
        self.min_speed = 0.05         # Minimum linear speed when approaching obstacles
        self.max_rotation_speed = 1.5 # Maximum rotation speed (rad/s)
        self.normal_speed = 0.2       # Default speed
        
        # Scanning parameters
        self.scan_angle = 70         # Degrees to scan left/right (each side)
        self.scan_sections = 5        # Number of sections to divide front scan into
        
        self.tele_twist = Twist()
        self.tele_twist.linear.x = self.normal_speed
        self.tele_twist.angular.z = 0.0
        
        # State variables
        self.avoiding = False
        self.last_rotation_direction = 1  # 1 for right, -1 for left
        self.obstacle_detected = False
        self.safe_to_move = True

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        if not self.obstacle_detected:  # Only accept commands when path is clear
            self.tele_twist = msg

    def get_section_distances(self, center_angle, angle_width, num_sections):
        """Divide the scan area into sections and get min distance for each"""
        if not self.scan_ranges:
            return [float('inf')] * num_sections
            
        num_ranges = len(self.scan_ranges)
        center_index = num_ranges // 2
        scan_width = int(angle_width * num_ranges / 360)
        
        section_ranges = []
        section_size = scan_width // num_sections
        
        for i in range(num_sections):
            start_idx = center_index - scan_width//2 + i*section_size
            end_idx = start_idx + section_size
            section = self.scan_ranges[start_idx:end_idx]
            section_ranges.append(min(section) if section else float('inf'))
            
        return section_ranges

    def analyze_front_sector(self):
        """Analyze the front sector in multiple sections"""
        if not self.scan_ranges:
            return float('inf'), 0, [float('inf')] * self.scan_sections
            
        # Get distances for each section
        section_distances = self.get_section_distances(
            center_angle=0,
            angle_width=self.scan_angle*2,
            num_sections=self.scan_sections
        )
        
        min_distance = min(section_distances)
        
        # Find which sections are blocked
        blocked_sections = [i for i, dist in enumerate(section_distances) 
                          if dist < self.clearance_distance]
        
        # Determine best turn direction
        if blocked_sections:
            # More blocked sections on the left suggests turning right
            left_blocks = sum(1 for i in blocked_sections if i < self.scan_sections//2)
            right_blocks = len(blocked_sections) - left_blocks
            turn_direction = -1 if left_blocks < right_blocks else 1
        else:
            turn_direction = 0
            
        return min_distance, turn_direction, section_distances

    def timer_callback(self):
        if self.has_scan_received:
            self.avoid_obstacles()

    def avoid_obstacles(self):
        twist = Twist()
        
        # Analyze the front sector
        min_dist, turn_direction, section_distances = self.analyze_front_sector()
        self.obstacle_detected = min_dist < self.clearance_distance
        
        # Emergency stop check
        if min_dist < self.emergency_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.avoiding = True
            self.get_logger().warn('EMERGENCY STOP! Too close to obstacle!', throttle_duration_sec=1)
        
        # Normal obstacle handling
        elif min_dist < self.stop_distance:
            # Stop and rotate to find clear path
            twist.linear.x = 0.0
            if turn_direction != 0:
                twist.angular.z = turn_direction * self.max_rotation_speed * 0.7
                self.last_rotation_direction = turn_direction
                self.avoiding = True
                self.get_logger().info('Obstacle detected! Stopping and turning.', throttle_duration_sec=1)
            else:
                # No clear direction - rotate in last direction
                twist.angular.z = self.last_rotation_direction * self.max_rotation_speed * 0.7
                self.get_logger().info('No clear path! Continuing rotation.', throttle_duration_sec=1)
                
        elif min_dist < self.slowdown_distance:
            # Slow down and prepare to turn
            speed_ratio = (min_dist - self.stop_distance) / (self.slowdown_distance - self.stop_distance)
            current_speed = max(self.min_speed, self.normal_speed * speed_ratio)
            
            if turn_direction != 0:
                # Gradually turn while moving
                twist.linear.x = current_speed
                turn_speed = self.max_rotation_speed * (1 - speed_ratio) * 0.5
                twist.angular.z = turn_direction * turn_speed
                self.last_rotation_direction = turn_direction
                self.avoiding = True
                self.get_logger().info('Approaching obstacle. Slowing to {:.2f}m/s and turning.'.format(current_speed), 
                                     throttle_duration_sec=1)
            else:
                # Just slow down
                twist.linear.x = current_speed
                twist.angular.z = 0.0
                self.get_logger().info('Approaching obstacle. Slowing to {:.2f}m/s.'.format(current_speed), 
                                     throttle_duration_sec=1)
                
        else:
            # Path is clear - resume normal operation
            if self.avoiding:
                self.get_logger().info('Path clear. Resuming normal movement.', throttle_duration_sec=1)
                self.avoiding = False
            twist = self.tele_twist

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()