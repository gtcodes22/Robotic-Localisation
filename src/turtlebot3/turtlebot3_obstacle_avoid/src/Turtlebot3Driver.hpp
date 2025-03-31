#ifndef TURTLEBOT3_DRIVER_HPP_
#define TURTLEBOT3_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Turtlebot3Driver : public rclcpp::Node {
public:
    Turtlebot3Driver();
    void moveForward();
    void stopMoving();
    void turn(double angular);
    bool obstacleDetected() const;

private:
    static constexpr double FORWARD_SPEED = 0.2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub;
    bool isObstacleDetected;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif  // TURTLEBOT3_DRIVER_HPP_

