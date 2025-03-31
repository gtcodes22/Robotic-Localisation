import rclpy
import numpy as np
import heapq
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from collections import deque

class PIDController:
    def __init__(self, kp_linear=0.5, ki_linear=0.01, kd_linear=0.1,
                 kp_angular=1.0, ki_angular=0.05, kd_angular=0.2):
        # Linear PID gains
        self.kp_linear = kp_linear
        self.ki_linear = ki_linear
        self.kd_linear = kd_linear
        
        # Angular PID gains
        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular
        
        # Error accumulators (for integral term)
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        
        # Previous errors (for derivative term)
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0

    def compute_velocity(self, current_pos, goal_pos, dt=0.1):
        """Compute PID-controlled velocities."""
        dx = goal_pos[0] - current_pos[0]
        dy = goal_pos[1] - current_pos[1]
        
        # Distance error (linear)
        distance_error = np.sqrt(dx**2 + dy**2)
        
        # Angle error (angular)
        angle_error = np.arctan2(dy, dx)
        
        # --- PID for Linear Velocity ---
        self.integral_linear += distance_error * dt
        derivative_linear = (distance_error - self.prev_error_linear) / dt
        linear_vel = (
            self.kp_linear * distance_error +
            self.ki_linear * self.integral_linear +
            self.kd_linear * derivative_linear
        )
        self.prev_error_linear = distance_error
        
        # --- PID for Angular Velocity ---
        self.integral_angular += angle_error * dt
        derivative_angular = (angle_error - self.prev_error_angular) / dt
        angular_vel = (
            self.kp_angular * angle_error +
            self.ki_angular * self.integral_angular +
            self.kd_angular * derivative_angular
        )
        self.prev_error_angular = angle_error
        
        return linear_vel, angular_vel


class AStarPathplanner(Node):
    def __init__(self):
        super().__init__('a_star_pathplanner')
        
        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(PoseStamped, '/current_waypoint', 10)
        
        # Initialize PID Controller
        self.controller = PIDController(
            kp_linear=0.5, ki_linear=0.01, kd_linear=0.1,
            kp_angular=1.0, ki_angular=0.05, kd_angular=0.2
        )
        
        # Internal variables
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.robot_position = None
        self.goal_position = (20, 20)  # Example goal (grid coordinates)
        self.obstacles = set()
        
        # Path tracking
        self.current_goal = None
        self.waypoints = deque()
        self.replan_threshold = 0.3  # Replan if obstacle is within 30cm
        
        # Smoothing
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.max_accel_linear = 0.1  # m/s²
        self.max_accel_angular = 0.5  # rad/s²
        
        # Control loop (10Hz)
        self.create_timer(0.1, self.control_loop)
        
        # Replanning check (2Hz)
        self.create_timer(0.5, self.check_for_obstacles)

    def map_callback(self, msg):
        """Process map data."""
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        width, height = msg.info.width, msg.info.height
        grid = np.array(msg.data).reshape((height, width))
        grid[grid < 0] = 1  # Treat unknown as obstacles
        self.map_data = grid
        self.get_logger().info("Map received and processed.")

    def odom_callback(self, msg):
        """Update robot position."""
        if self.origin is None or self.resolution is None:
            self.get_logger().warn("Waiting for map data...")
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_position = self.world_to_grid(x, y)
        self.get_logger().info(f"Robot position updated: {self.robot_position}")

    def scan_callback(self, msg):
        """Detect dynamic obstacles."""
        if self.robot_position is None:
            self.get_logger().warn("Waiting for odometry data...")
            return  # Exit function if robot position is not available

        self.obstacles.clear()
        angle_min, angle_increment = msg.angle_min, msg.angle_increment
        for i, distance in enumerate(msg.ranges):
            if 0.1 < distance < 1.0:  # Ignore too close/far readings
                angle = angle_min + i * angle_increment
                ox = int(self.robot_position[0] + (distance * np.cos(angle) / self.resolution))
                oy = int(self.robot_position[1] + (distance * np.sin(angle) / self.resolution))
                self.obstacles.add((ox, oy))

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        if self.origin is None or self.resolution is None:
            self.get_logger().error("Map data not available!")
            return (0, 0)
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return (gx, gy)

    def a_star(self, start, goal):
        """A* Pathplanning Algorithm."""
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-directional
                neighbor = (current[0] + dx, current[1] + dy)

                if (0 <= neighbor[0] < self.map_data.shape[0] and
                    0 <= neighbor[1] < self.map_data.shape[1] and
                    self.map_data[neighbor] == 0 and
                    neighbor not in self.obstacles):

                    temp_g = g_score[current] + 1
                    if neighbor not in g_score or temp_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = temp_g
                        f_score[neighbor] = temp_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def reconstruct_path(self, came_from, current):
        """Reconstruct path from goal to start."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def check_for_obstacles(self):
        """Trigger replanning if obstacles are near the path."""
        if not self.waypoints or not self.robot_position:
            return
        
        # Check if next waypoint is blocked
        next_waypoint = self.waypoints[0]
        if next_waypoint in self.obstacles:
            self.get_logger().warn("Obstacle detected on path! Replanning...")
            self.navigate_to_goal()

    def navigate_to_goal(self):
        """Compute A* path and set waypoints."""
        if self.map_data is None or self.robot_position is None:
            self.get_logger().warn("Waiting for map/robot data...")
            return
        
        path = self.a_star(self.robot_position, self.goal_position)
        if path:
            self.waypoints = deque(path)
            self.current_goal = self.waypoints.popleft()
            self.get_logger().info(f"New path with {len(path)} waypoints.")
        else:
            self.get_logger().warn("No valid path found!")

    def control_loop(self):
        """Execute PID control with velocity smoothing."""
        if not self.current_goal or not self.robot_position:
            return
        
        # Compute raw PID velocities
        linear_vel, angular_vel = self.controller.compute_velocity(
            self.robot_position,
            self.current_goal
        )
        
        # Apply velocity smoothing (acceleration limits)
        linear_vel = np.clip(
            linear_vel,
            self.last_linear_vel - self.max_accel_linear * 0.1,
            self.last_linear_vel + self.max_accel_linear * 0.1
        )
        angular_vel = np.clip(
            angular_vel,
            self.last_angular_vel - self.max_accel_angular * 0.1,
            self.last_angular_vel + self.max_accel_angular * 0.1
        )
        
        # Publish smoothed velocities
        twist = Twist()
        twist.linear.x = np.clip(linear_vel, -0.2, 0.2)
        twist.angular.z = np.clip(angular_vel, -1.0, 1.0)
        self.cmd_vel_pub.publish(twist)
        self.last_linear_vel = linear_vel
        self.last_angular_vel = angular_vel
        
        # Check if waypoint reached
        distance = np.sqrt(
            (self.current_goal[0] - self.robot_position[0])**2 +
            (self.current_goal[1] - self.robot_position[1])**2
        )
        if distance < 0.1:  # 10cm threshold
            if self.waypoints:
                self.current_goal = self.waypoints.popleft()
            else:
                self.current_goal = None
                twist = Twist()  # Stop
                self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPathplanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()