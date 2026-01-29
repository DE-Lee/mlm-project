#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        
        # Parameters
        self.declare_parameter('drive_type', 'Ackermann')  # 'Ackermann' or 'Mecanum'
        self.declare_parameter('angular_p_gain', 2.0)
        self.declare_parameter('max_linear_velocity', 0.8)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('arrival_tolerance', 0.005)  # Hard stop threshold
        self.declare_parameter('distance_scale_factor', 1.0)  # Odometry correction factor
        self.declare_parameter('steering_offset_rad', 0.0)  # Steering bias correction
        
        self.drive_type = self.get_parameter('drive_type').value
        self.angular_p_gain = self.get_parameter('angular_p_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.arrival_tolerance = self.get_parameter('arrival_tolerance').value
        self.distance_scale_factor = self.get_parameter('distance_scale_factor').value
        self.steering_offset_rad = self.get_parameter('steering_offset_rad').value
        
        # Subscribers
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/target_point',
            self.target_callback,
            10
        )
        
        self.velocity_sub = self.create_subscription(
            Float64,
            '/target_velocity',
            self.velocity_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher
        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 세이프티 모듈 활용 테스트용 코드로 대체
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        
        # State variables
        self.current_pose = None
        self.target_pose = None
        self.target_velocity = 0.0
        self.current_yaw = 0.0
        self.has_target = False  # Flag to indicate if target has been received
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Motion Controller initialized (Drive Type: {self.drive_type})')
    
    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_pose = msg.pose.pose
        self.current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
    
    def target_callback(self, msg):
        """Update target pose"""
        self.target_pose = msg
        self.has_target = True  # Mark that we have received a target
    
    def velocity_callback(self, msg):
        """Update target velocity"""
        self.target_velocity = msg.data
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        """Send complete stop command to robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def publish_cmd(self, linear_velocity, angular_velocity):
        """Publish velocity command based on drive type"""
        cmd = Twist()
        
        if self.drive_type == 'Ackermann':
            # Ackermann drive: only forward/backward and rotation
            cmd.linear.x = linear_velocity
            cmd.angular.z = angular_velocity
        elif self.drive_type == 'Mecanum':
            # Mecanum drive: can move in any direction
            # For simplicity, we use same control as Ackermann
            # More sophisticated control could use lateral velocity
            cmd.linear.x = linear_velocity
            cmd.linear.y = 0.0  # Could be used for lateral movement
            cmd.angular.z = angular_velocity
        else:
            self.get_logger().warn(f'Unknown drive type: {self.drive_type}')
            return
        
        self.cmd_vel_pub.publish(cmd)
    
    def control_loop(self):
        """Main control loop implementing Pure Pursuit"""
        # Stop if no valid data
        if self.current_pose is None or self.target_pose is None or not self.has_target:
            self.stop_robot()
            return
        
        # Calculate position error (distance)
        dx = self.target_pose.pose.position.x - self.current_pose.position.x
        dy = self.target_pose.pose.position.y - self.current_pose.position.y
        distance_error = math.sqrt(dx**2 + dy**2)
        
        # Strong arrival detection: hard stop when very close to target
        # Slightly more generous than server's goal_tolerance (0.05) for stability
        if distance_error < self.arrival_tolerance:
            # Near goal point - immediately issue physical stop command
            self.get_logger().debug(f"Within arrival tolerance ({self.arrival_tolerance}m). Hard Stop.")
            self.stop_robot()
            return
        
        # If server sends near-zero velocity, it means stop command
        if abs(self.target_velocity) < 0.01:
            self.get_logger().debug("Target velocity near zero. Stopping.")
            self.stop_robot()
            return
        
        # NOTE: Motion controller should NEVER stop based on distance
        # Path player server manages waypoint transitions
        # Motion controller only tracks the current target_point
        
        # Detect reverse mode from target velocity sign
        is_reverse = self.target_velocity < 0
        
        # Calculate angle error
        target_angle = math.atan2(dy, dx)
        
        if is_reverse:
            # In reverse mode, robot should face AWAY from target (180 degrees opposite)
            target_angle = self.normalize_angle(target_angle + math.pi)
        
        angle_error = self.normalize_angle(target_angle - self.current_yaw)
        
        # Distance-based velocity profiling for smooth acceleration/deceleration
        # NOTE: Don't slow down based on distance to target_point
        # because path_player constantly updates target_point (look-ahead)
        # Only slow down based on angle error for sharp turns
        distance_based_velocity = self.target_velocity
        
        # Slow down when angle error is large (> 30 degrees) - helps with sharp turns
        angle_error_abs = abs(angle_error)
        if angle_error_abs > 0.52:  # ~30 degrees
            angle_reduction = 1.0 - min((angle_error_abs - 0.52) / 1.05, 0.7)  # Reduce up to 70%
            distance_based_velocity *= angle_reduction
        
        linear_velocity = distance_based_velocity
        
        # Apply distance scale factor correction
        # If robot travels 87cm when commanded 1m, use factor 1.15 (1.0/0.87)
        linear_velocity *= self.distance_scale_factor
        
        # Apply velocity limits
        linear_velocity = max(min(linear_velocity, self.max_linear_vel), -self.max_linear_vel)
        
        # Calculate angular velocity with P-gain
        angular_velocity = self.angular_p_gain * angle_error
        angular_velocity = max(min(angular_velocity, self.max_angular_vel), -self.max_angular_vel)
        
        # Enhanced angular deadband: prevent fishtailing
        # Very small angle errors (< 3 degrees) are ignored
        strict_angle_deadband = math.radians(3)  # 3 degrees in radians
        if abs(angle_error) < strict_angle_deadband:
            angular_velocity = 0.0
        
        # Improved deadband logic
        distance_deadband = 0.02  # meters (reduced from 0.08 - too large!)
        angle_deadband = 0.087  # ~5 degrees in radians
        
        if distance_error < distance_deadband:
            # Very close to target - stop rotating to prevent oscillation
            # BUT: Do NOT stop linear velocity - path_player_server manages stopping
            angular_velocity = 0.0
        elif abs(angle_error) < angle_deadband:
            # Angle is already good - go straight
            angular_velocity = 0.0
        
        # Apply steering offset correction (after deadband to ensure it's always applied)
        # Negative if drifts right, positive if drifts left
        if angular_velocity != 0.0 or abs(self.steering_offset_rad) > 0.01:
            angular_velocity += self.steering_offset_rad
        
        # Debug logging
        self.get_logger().debug(
            f'[CONTROL] Distance: {distance_error:.3f}m | '
            f'Angle Error: {math.degrees(angle_error):.1f}° | '
            f'Linear: {linear_velocity:.3f} m/s | '
            f'Angular: {angular_velocity:.3f} rad/s'
        )
        
        # Publish command using helper method
        self.publish_cmd(linear_velocity, angular_velocity)


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
