#include "Turtlebot3Driver.hpp"

Turtlebot3Driver::Turtlebot3Driver() : Node("turtlebot3_driver"), isObstacleDetected(false) {
    commandPub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laserSub = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&Turtlebot3Driver::scanCallback, this, std::placeholders::_1));
}

void Turtlebot3Driver::moveForward() {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x = FORWARD_SPEED;
    commandPub->publish(std::move(msg));
}

void Turtlebot3Driver::stopMoving() {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    commandPub->publish(std::move(msg));
}

void Turtlebot3Driver::turn(double angular) {
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->angular.z = angular;
    commandPub->publish(std::move(msg));
}

bool Turtlebot3Driver::obstacleDetected() const {
    return isObstacleDetected;
}

void Turtlebot3Driver::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    const double ANGLE_RANGE = 0.032;
    const double MIN_DIST_FROM_OBSTACLE = 0.8;

    double angle_min = scan->angle_min;
    double angle_increment = scan->angle_increment;
    int start_index = static_cast<int>((-ANGLE_RANGE / 2 - angle_min) / angle_increment);
    int end_index = static_cast<int>((ANGLE_RANGE / 2 - angle_min) / angle_increment);

    for (int i = start_index; i <= end_index; ++i) {
        if (scan->ranges[i] < MIN_DIST_FROM_OBSTACLE) {
            isObstacleDetected = true;
            return;
        }
    }
    isObstacleDetected = false;
}

