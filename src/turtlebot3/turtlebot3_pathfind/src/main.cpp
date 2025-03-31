#include "rclcpp/rclcpp.hpp"
#include "Turtlebot3Driver.hpp"
#include "AStarPlanner.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class Turtlebot3Navigator : public rclcpp::Node {
public:
    Turtlebot3Navigator() : Node("turtlebot3_navigator"), 
                          planner_(100, 100),
                          tf_buffer_(this->get_clock()),
                          tf_listener_(tf_buffer_) {
        
        // Initialize parameters
        this->declare_parameter("linear_speed", 0.1);
        this->declare_parameter("angular_speed", 0.5);
        this->declare_parameter("goal_tolerance", 0.1);
        
        // Subscribers
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&Turtlebot3Navigator::mapCallback, this, std::placeholders::_1));
            
        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&Turtlebot3Navigator::goalCallback, this, std::placeholders::_1));
        
        // Controller timer
        controller_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Turtlebot3Navigator::controlLoop, this));
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Convert map to binary grid for A*
        grid_.resize(msg->info.width, std::vector<bool>(msg->info.height, false));
        for (size_t i = 0; i < msg->data.size(); i++) {
            int x = i % msg->info.width;
            int y = i / msg->info.width;
            grid_[x][y] = (msg->data[i] > 50); // Consider cells >50% occupied as obstacles
        }
        map_resolution_ = msg->info.resolution;
        map_origin_x_ = msg->info.origin.position.x;
        map_origin_y_ = msg->info.origin.position.y;
        has_map_ = true;
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = *msg;
        has_goal_ = true;
        planPath();
    }
    
    void planPath() {
        if (!has_map_ || !has_goal_) return;
        
        try {
            // Get current pose in map frame
            auto transform = tf_buffer_.lookupTransform(
                "map", "base_link",
                this->now(), rclcpp::Duration::from_seconds(0.1));
            
            int start_x = static_cast<int>((transform.transform.translation.x - map_origin_x_) / map_resolution_);
            int start_y = static_cast<int>((transform.transform.translation.y - map_origin_y_) / map_resolution_);
            int goal_x = static_cast<int>((goal_pose_.pose.position.x - map_origin_x_) / map_resolution_);
            int goal_y = static_cast<int>((goal_pose_.pose.position.y - map_origin_y_) / map_resolution_);
            
            auto path = planner_.findPath(start_x, start_y, goal_x, goal_y, grid_);
            if (!path.empty()) {
                path_commands_ = planner_.convertPathToCommands(path, map_resolution_);
                current_command_index_ = 0;
                RCLCPP_INFO(this->get_logger(), "New path planned with %zu commands", path_commands_.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "No path found to goal!");
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
        }
    }
    
    void controlLoop() {
        if (path_commands_.empty() || current_command_index_ >= path_commands_.size()) {
            driver_->stopMoving();
            return;
        }
        
        auto& cmd = path_commands_[current_command_index_];
        float linear_speed = this->get_parameter("linear_speed").as_double();
        float angular_speed = this->get_parameter("angular_speed").as_double();
        float goal_tolerance = this->get_parameter("goal_tolerance").as_double();
        
        // Check for obstacles
        if (driver_->obstacleDetected()) {
            driver_->stopMoving();
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Replanning...");
            planPath(); // Replan
            return;
        }
        
        // Execute current command
        switch (cmd.type) {
            case PathSegment::FORWARD: {
                driver_->moveForward(linear_speed);
                cmd.distance -= linear_speed * 0.1; // 100ms control loop
                break;
            }
            case PathSegment::TURN_LEFT: {
                driver_->turn(angular_speed);
                cmd.distance -= angular_speed * 0.1;
                break;
            }
            case PathSegment::TURN_RIGHT: {
                driver_->turn(-angular_speed);
                cmd.distance -= angular_speed * 0.1;
                break;
            }
        }
        
        // Check if command is complete
        if (cmd.distance <= 0) {
            current_command_index_++;
            if (current_command_index_ < path_commands_.size()) {
                RCLCPP_INFO(this->get_logger(), "Starting next command: %d", 
                          path_commands_[current_command_index_].type);
            }
        }
        
        // Check if reached goal
        if (current_command_index_ >= path_commands_.size()) {
            try {
                auto transform = tf_buffer_.lookupTransform(
                    "map", "base_link",
                    this->now(), rclcpp::Duration::from_seconds(0.1));
                
                float dx = goal_pose_.pose.position.x - transform.transform.translation.x;
                float dy = goal_pose_.pose.position.y - transform.transform.translation.y;
                float distance = sqrt(dx*dx + dy*dy);
                
                if (distance < goal_tolerance) {
                    RCLCPP_INFO(this->get_logger(), "Goal reached!");
                    path_commands_.clear();
                } else {
                    RCLCPP_INFO(this->get_logger(), "Final approach needed");
                    // Add final approach commands
                    float angle = atan2(dy, dx);
                    path_commands_.push_back({PathSegment::TURN_LEFT, angle});
                    path_commands_.push_back({PathSegment::FORWARD, distance});
                    current_command_index_ = path_commands_.size() - 2;
                }
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
            }
        }
    }
    
    void setDriver(std::shared_ptr<Turtlebot3Driver> driver) {
        driver_ = driver;
    }

private:
    AStarPlanner planner_;
    std::vector<std::vector<bool>> grid_;
    float map_resolution_;
    float map_origin_x_, map_origin_y_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    bool has_map_ = false;
    bool has_goal_ = false;
    std::shared_ptr<Turtlebot3Driver> driver_;
    
    // Path following
    std::vector<PathSegment> path_commands_;
    size_t current_command_index_ = 0;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // ROS2
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto driver_node = std::make_shared<Turtlebot3Driver>();
    auto navigator_node = std::make_shared<Turtlebot3Navigator>();
    navigator_node->setDriver(driver_node);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(driver_node);
    executor.add_node(navigator_node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}