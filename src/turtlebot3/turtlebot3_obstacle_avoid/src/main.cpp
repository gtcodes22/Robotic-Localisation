#include "rclcpp/rclcpp.hpp"
#include "Turtlebot3Driver.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Turtlebot3Driver>();
    while (rclcpp::ok()) {
        // ------------------------------ FUNCTIONS AVAILABLE -----------------------------
        // node->moveForward() - Moves the robot forward at a given speed
        // node->stopMoving() - Stops the robot in its current position
        // node->obstacleDetected() - Returns true or false if obstacle within threshold
        // node->turn(1.0) - Turns the robot counterclockwise at speed 1 rad/s
        // node->turn(-1,0) - Turns the robot clockwise at speed 1 rad/s
        // ----------------------------- ALGORITHM GOES HERE -----------------------------
        
        int stop_time = 1000;
        int turn_time = 1000;
   	
    		if (node->obstacleDetected()){
    			node->stopMoving();
    			rclcpp::sleep_for(std::chrono::milliseconds(stop_time)); // Remain stationary for 1 second
    			node->turn(-1.0); // Turn right
    			rclcpp::sleep_for(std::chrono::milliseconds(turn_time)); // Sleep for 1 second while turning
    			node->turn(0.0); // Stop turning
    			}
    		else {
    			node->moveForward();
    		}
    		
        // --------------------------------------------------------------------------------
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}

