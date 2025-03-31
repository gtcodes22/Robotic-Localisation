// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "turtlebot3_drive/turtlebot3_drive.hpp"

#include <memory>

using namespace std::chrono_literals;

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables with safer defaults for real robot
  ************************************************************/
 scan_data_[0] = 0.0;
 scan_data_[1] = 0.0;
 scan_data_[2] = 0.0;

 robot_pose_ = 0.0;
 prev_robot_pose_ = 0.0;

 // Reduced velocities for real robot safety
 linear_velocity_ = 0.15;  // Reduced from simulation value
 angular_velocity_ = 0.5;  // Reduced from simulation value

 /************************************************************
 ** Initialise ROS publishers and subscribers
 ************************************************************/
 auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

 // Initialise publishers
 cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

 // Initialise subscribers with proper topic names for real robot
 scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
   "scan",  // Real TurtleBot3 typically uses "scan" for LIDAR
   rclcpp::SensorDataQoS(), 
   std::bind(&Turtlebot3Drive::scan_callback, this, std::placeholders::_1));
   
 odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
   "odom",  // Real TurtleBot3 typically uses "odom"
   qos, 
   std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

 // Add emergency stop subscriber
 emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
   "emergency_stop",
   qos,
   std::bind(&Turtlebot3Drive::emergency_stop_callback, this, std::placeholders::_1));

 /************************************************************
 ** Initialise ROS timers
 ************************************************************/
 update_timer_ = this->create_wall_timer(50ms, std::bind(&Turtlebot3Drive::update_callback, this));  // Slower update rate

 RCLCPP_INFO(this->get_logger(), "Turtlebot3 real robot node has been initialised");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
 // Stop the robot when shutting down
 update_cmd_vel(0.0, 0.0);
 RCLCPP_INFO(this->get_logger(), "Turtlebot3 real robot node has been terminated");
}

/********************************************************************************
** New emergency stop callback
********************************************************************************/
void Turtlebot3Drive::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
 if (msg->data) {
   update_cmd_vel(0.0, 0.0);
   RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
   // You might want to add additional emergency handling here
 }
}

/********************************************************************************
** Modified callback functions for real robot
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
 // Add more robust orientation handling
 try {
   tf2::Quaternion q(
     msg->pose.pose.orientation.x,
     msg->pose.pose.orientation.y,
     msg->pose.pose.orientation.z,
     msg->pose.pose.orientation.w);
   tf2::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);

   robot_pose_ = yaw;
 } catch (tf2::TransformException &ex) {
   RCLCPP_WARN(this->get_logger(), "Odom transform exception: %s", ex.what());
 }
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
 // Use wider angle ranges for more robust detection on real robot
 uint16_t scan_angle[3] = {0, 45, 315};  // Wider angles than simulation

 for (int num = 0; num < 3; num++) {
   // Add more comprehensive checks for real sensor data
   if (std::isinf(msg->ranges.at(scan_angle[num])) {
     scan_data_[num] = msg->range_max;
   } else if (std::isnan(msg->ranges.at(scan_angle[num]))) {
     scan_data_[num] = msg->range_max;
   } else if (msg->ranges.at(scan_angle[num]) < msg->range_min) {
     scan_data_[num] = msg->range_min;
   } else {
     scan_data_[num] = msg->ranges.at(scan_angle[num]);
   }
 }
}

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
 // Add velocity limiting for safety
 const double MAX_LINEAR = 0.22;  // TurtleBot3 Burger max safe linear velocity
 const double MAX_ANGULAR = 2.84; // TurtleBot3 Burger max safe angular velocity

 linear = std::clamp(linear, -MAX_LINEAR, MAX_LINEAR);
 angular = std::clamp(angular, -MAX_ANGULAR, MAX_ANGULAR);

 geometry_msgs::msg::Twist cmd_vel;
 cmd_vel.linear.x = linear;
 cmd_vel.angular.z = angular;

 cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Modified update function for real robot
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
 if (emergency_stop_activated_) {
   update_cmd_vel(0.0, 0.0);
   return;
 }

 static uint8_t turtlebot3_state_num = 0;
 double escape_range = 45.0 * DEG2RAD;  // Increased from simulation
 double check_forward_dist = 0.5;       // More conservative distance
 double check_side_dist = 0.4;          // More conservative distance

 switch (turtlebot3_state_num) {
   case GET_TB3_DIRECTION:
     if (scan_data_[CENTER] > check_forward_dist) {
       if (scan_data_[LEFT] < check_side_dist) {
         prev_robot_pose_ = robot_pose_;
         turtlebot3_state_num = TB3_RIGHT_TURN;
       } else if (scan_data_[RIGHT] < check_side_dist) {
         prev_robot_pose_ = robot_pose_;
         turtlebot3_state_num = TB3_LEFT_TURN;
       } else {
         turtlebot3_state_num = TB3_DRIVE_FORWARD;
       }
     } else {
       prev_robot_pose_ = robot_pose_;
       turtlebot3_state_num = TB3_RIGHT_TURN;
     }
     break;

   case TB3_DRIVE_FORWARD:
     update_cmd_vel(linear_velocity_, 0.0);
     turtlebot3_state_num = GET_TB3_DIRECTION;
     break;

   case TB3_RIGHT_TURN:
     if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
       turtlebot3_state_num = GET_TB3_DIRECTION;
     } else {
       update_cmd_vel(0.0, -angular_velocity_);
     }
     break;

   case TB3_LEFT_TURN:
     if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
       turtlebot3_state_num = GET_TB3_DIRECTION;
     } else {
       update_cmd_vel(0.0, angular_velocity_);
     }
     break;

   default:
     turtlebot3_state_num = GET_TB3_DIRECTION;
     break;
 }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>());
  rclcpp::shutdown();

  return 0;
}
