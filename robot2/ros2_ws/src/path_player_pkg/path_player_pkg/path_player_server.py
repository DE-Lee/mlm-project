#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from path_player_pkg.action import FollowPath
import csv
import math
import os
from ament_index_python.packages import get_package_share_directory


class PathPlayerServer(Node):
    def __init__(self):
        super().__init__('path_player_server')
        
        # Get package share directory for data files
        package_share_dir = get_package_share_directory('path_player_pkg')
        default_csv_dir = os.path.join(package_share_dir, 'data')
        
        # Parameters
        self.declare_parameter('csv_directory', default_csv_dir)
        self.declare_parameter('goal_tolerance', 0.005)
        self.declare_parameter('start_tolerance', 1.0)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('look_ahead_distance', 0.6)  # Look-ahead distance (Ld) - increased for smoother motion
        self.declare_parameter('min_look_ahead', 0.2)
        self.declare_parameter('max_look_ahead', 0.8)
        self.declare_parameter('default_velocity', 0.5)  # Default velocity when CSV doesn't have velocity column
        
        self.csv_directory = self.get_parameter('csv_directory').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.start_tolerance = self.get_parameter('start_tolerance').value
        self.update_rate = self.get_parameter('update_rate').value
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.min_look_ahead = self.get_parameter('min_look_ahead').value
        self.max_look_ahead = self.get_parameter('max_look_ahead').value
        self.default_velocity = self.get_parameter('default_velocity').value
        
        # Action Server
        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # QoS Profile for odometry (BEST_EFFORT to match robot publisher)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 발행자(로봇) 설정에 맞춤
            depth=10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile  # 기본값 10 대신 qos_profile 적용
        )
        
        # Publishers
        self.target_point_pub = self.create_publisher(
            PoseStamped,
            '/target_point',
            10
        )
        
        self.target_velocity_pub = self.create_publisher(
            Float64,
            '/target_velocity',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/global_path',
            10
        )
        
        # State variables
        self.current_pose = None
        self.path_data = None
        self.current_index = 0
        self.reverse_mode = False
        
        self.get_logger().info('Path Player Server initialized')
    
    def odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        self.get_logger().info(f'Received goal request: path={goal_request.path_name}, reverse={goal_request.reverse}')
        
        # Check if CSV file exists (add .csv extension if not present)
        csv_filename = goal_request.path_name if goal_request.path_name.endswith('.csv') else f'{goal_request.path_name}.csv'
        csv_path = os.path.join(self.csv_directory, csv_filename)
        
        if not os.path.exists(csv_path):
            self.get_logger().error(f'CSV file not found: {csv_path}')
            return GoalResponse.REJECT
        
        self.get_logger().info(f'CSV file found: {csv_path}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def load_path(self, path_name):
        """Load path data from CSV file (supports multiple formats)"""
        # Add .csv extension if not present
        csv_filename = path_name if path_name.endswith('.csv') else f'{path_name}.csv'
        csv_path = os.path.join(self.csv_directory, csv_filename)
        
        try:
            with open(csv_path, 'r') as f:
                # Read first line to check if it's a header
                first_line = f.readline().strip()
                f.seek(0)  # Reset to beginning
                
                # Check if first line is a header (contains non-numeric characters except minus/dot)
                first_values = first_line.split(',')
                is_header = False
                try:
                    # Try to parse first value as float
                    float(first_values[0])
                except ValueError:
                    # If it fails, it's likely a header
                    is_header = True
                
                if is_header:
                    # Use DictReader for files with header
                    reader = csv.DictReader(f)
                    data = list(reader)
                    if not data:
                        self.get_logger().error('CSV file is empty')
                        return None
                    columns = data[0].keys()
                else:
                    # Handle files without header: use only first 2 columns (x, y)
                    self.get_logger().info('No header detected, using first 2 columns as x, y')
                    reader = csv.reader(f)
                    raw_data = list(reader)
                    if not raw_data:
                        self.get_logger().error('CSV file is empty')
                        return None
                    
                    # Convert to dict format using only first 2 columns
                    data = []
                    columns = ['x', 'y']
                    for row_values in raw_data:
                        if len(row_values) < 2:
                            self.get_logger().error(f'Row has less than 2 columns: {row_values}')
                            return None
                        data.append({
                            'x': row_values[0],
                            'y': row_values[1]
                        })
            
            # Check available columns
            columns = list(data[0].keys()) if data else []
            
            # Verify minimum required columns (x, y)
            if 'x' not in columns or 'y' not in columns:
                self.get_logger().error('CSV must contain at least x and y columns')
                return None
            
            # Detect format and convert data
            has_velocity = 'velocity' in columns
            has_yaw = 'yaw' in columns
            has_odom_yaw = 'odom_yaw' in columns
            has_imu_yaw = 'imu_yaw' in columns
            
            # Log detected format
            if has_odom_yaw or has_imu_yaw:
                self.get_logger().info('Detected Waypoint Recorder format (with wall distances)')
            elif has_yaw and has_velocity:
                self.get_logger().info('Detected standard path player format')
            else:
                self.get_logger().info('Detected minimal format (x, y only)')
            
            # Process each waypoint
            for i, row in enumerate(data):
                row['x'] = float(row['x'])
                row['y'] = float(row['y'])
                
                # Handle yaw: priority order - yaw > odom_yaw > imu_yaw > calculated
                if has_yaw:
                    row['yaw'] = float(row['yaw'])
                elif has_odom_yaw:
                    row['yaw'] = float(row['odom_yaw'])
                elif has_imu_yaw:
                    row['yaw'] = float(row['imu_yaw'])
                else:
                    # Calculate yaw from position difference
                    if i == 0:
                        # First waypoint: use next waypoint direction or 0
                        if len(data) > 1:
                            dx = float(data[1]['x']) - row['x']
                            dy = float(data[1]['y']) - row['y']
                            row['yaw'] = math.atan2(dy, dx)
                        else:
                            row['yaw'] = 0.0
                    else:
                        # Use direction from previous waypoint
                        dx = row['x'] - float(data[i-1]['x'])
                        dy = row['y'] - float(data[i-1]['y'])
                        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                            # Same position, use previous yaw
                            row['yaw'] = data[i-1]['yaw']
                        else:
                            row['yaw'] = math.atan2(dy, dx)
                
                # Handle velocity
                if has_velocity:
                    row['velocity'] = float(row['velocity'])
                else:
                    # Use default velocity from parameter
                    row['velocity'] = self.default_velocity
            
            self.get_logger().info(f'Loaded path with {len(data)} waypoints')
            self.get_logger().info(f'Format: x, y, yaw={"provided" if (has_yaw or has_odom_yaw or has_imu_yaw) else "calculated"}, velocity={"provided" if has_velocity else f"default({self.default_velocity})"}')
            
            return data
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None
    
    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def find_closest_point_index(self, robot_x, robot_y):
        """Find the index of the closest waypoint to the robot
        
        IMPORTANT: Searches forward/backward from current_index based on reverse mode.
        This ensures the robot always progresses along the path.
        """
        min_distance = float('inf')
        closest_index = self.current_index
        
        # Search in the direction of progress with small safety window
        if self.reverse_mode:
            # Reverse mode: search backward (decreasing index)
            # Allow +3 ahead (higher index) for safety, search down to 0
            search_start = min(len(self.path_data) - 1, self.current_index + 3)
            search_end = max(0, self.current_index - 10)  # Look 10 waypoints back max
            search_range = range(search_start, search_end - 1, -1)
        else:
            # Forward mode: search forward (increasing index)
            # Allow -3 back for safety, search up to end
            search_start = max(0, self.current_index - 3)
            search_end = len(self.path_data)
            search_range = range(search_start, search_end)
        
        for i in search_range:
            waypoint = self.path_data[i]
            distance = self.calculate_distance(robot_x, robot_y, waypoint['x'], waypoint['y'])
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        self.get_logger().debug(
            f'[CLOSEST] Index: {closest_index} | '
            f'Distance: {min_distance:.3f}m | '
            f'Robot: ({robot_x:.2f}, {robot_y:.2f}) | '
            f'Waypoint: ({self.path_data[closest_index]["x"]:.2f}, {self.path_data[closest_index]["y"]:.2f}) | '
            f'Search range: [{search_start}-{len(self.path_data)-1}]'
        )
        
        return closest_index
    
    def find_look_ahead_point(self, robot_x, robot_y, start_index):
        """
        Find look-ahead target point on the path
        
        Args:
            robot_x: Current robot x position
            robot_y: Current robot y position
            start_index: Starting waypoint index (must search forward/backward from here based on reverse mode)
        
        Returns:
            tuple: (target_x, target_y, target_yaw, target_velocity, index)
        """
        # Determine final waypoint based on reverse mode
        if self.reverse_mode:
            final_wp = self.path_data[0]
            final_index = 0
        else:
            final_wp = self.path_data[-1]
            final_index = len(self.path_data) - 1
        
        distance_to_final = self.calculate_distance(robot_x, robot_y, final_wp['x'], final_wp['y'])
        
        # Reduce look-ahead distance when close to final goal
        is_adaptive = distance_to_final < self.look_ahead_distance * 2
        if is_adaptive:
            look_ahead_dist = max(
                self.goal_tolerance,
                distance_to_final * 0.5
            )
            self.get_logger().debug(
                f'[ADAPTIVE Ld] Distance to final: {distance_to_final:.3f}m | '
                f'Adaptive Ld: {look_ahead_dist:.3f}m (reduced from {self.look_ahead_distance:.3f}m)'
            )
        else:
            look_ahead_dist = self.look_ahead_distance
            self.get_logger().debug(
                f'[NORMAL Ld] Distance to final: {distance_to_final:.3f}m | '
                f'Normal Ld: {look_ahead_dist:.3f}m'
            )
        
        # Search direction based on reverse mode
        accumulated_dist = 0.0
        prev_x = robot_x
        prev_y = robot_y
        
        if self.reverse_mode:
            # Reverse mode: search backward (decreasing index)
            search_range = range(start_index, -1, -1)
        else:
            # Forward mode: search forward (increasing index)
            search_range = range(start_index, len(self.path_data))
        
        for i in search_range:
            wp = self.path_data[i]
            
            # Calculate distance from previous point
            segment_dist = self.calculate_distance(prev_x, prev_y, wp['x'], wp['y'])
            accumulated_dist += segment_dist
            
            # If accumulated distance exceeds look-ahead, this is our target
            if accumulated_dist >= look_ahead_dist:
                self.get_logger().debug(
                    f'[LOOKAHEAD] Target found at index {i}/{final_index} | '
                    f'Accumulated: {accumulated_dist:.3f}m >= Required: {look_ahead_dist:.3f}m | '
                    f'Target: ({wp["x"]:.2f}, {wp["y"]:.2f})'
                )
                return wp['x'], wp['y'], wp['yaw'], wp['velocity'], i
            
            prev_x = wp['x']
            prev_y = wp['y']
        
        # If we reach here, return the final waypoint
        self.get_logger().debug(
            f'[LOOKAHEAD] Reached path end, returning final waypoint | '
            f'Index: {final_index} | '
            f'Final: ({final_wp["x"]:.2f}, {final_wp["y"]:.2f})'
        )
        return final_wp['x'], final_wp['y'], final_wp['yaw'], final_wp['velocity'], final_index
    
    def publish_global_path(self):
        """Publish the current path as a Path message for visualization"""
        if self.path_data is None:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for waypoint in self.path_data:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = waypoint['x']
            pose.pose.position.y = waypoint['y']
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            yaw = waypoint['yaw']
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    async def execute_callback(self, goal_handle):
        """Execute the follow path action"""
        self.get_logger().info('Executing goal...')
        
        # Load path
        self.path_data = self.load_path(goal_handle.request.path_name)
        if self.path_data is None:
            goal_handle.abort()
            result = FollowPath.Result()
            result.success = False
            result.message = 'Failed to load path file'
            return result
        
        # Publish global path for visualization
        self.publish_global_path()
        self.get_logger().info('Published global path for visualization')
        
        # Setup reverse mode
        self.reverse_mode = goal_handle.request.reverse
        
        # Set initial index
        if self.reverse_mode:
            self.current_index = len(self.path_data) - 1
            final_index = 0
        else:
            self.current_index = 0
            final_index = len(self.path_data) - 1
        
        # Wait for odometry
        if self.current_pose is None:
            self.get_logger().warn('Waiting for odometry data...')
            timeout = 5.0
            elapsed = 0.0
            rate = self.create_rate(10)
            while self.current_pose is None and elapsed < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                elapsed += 0.1
            
            if self.current_pose is None:
                goal_handle.abort()
                result = FollowPath.Result()
                result.success = False
                result.message = 'No odometry data received'
                return result
        
        # Check if robot is close to start position
        start_x = self.path_data[self.current_index]['x']
        start_y = self.path_data[self.current_index]['y']
        distance_to_start = self.calculate_distance(
            self.current_pose.position.x,
            self.current_pose.position.y,
            start_x,
            start_y
        )
        
        self.get_logger().info(f'Robot position: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})')
        self.get_logger().info(f'Start position: ({start_x:.2f}, {start_y:.2f})')
        self.get_logger().info(f'Distance to start: {distance_to_start:.2f}m (tolerance: {self.start_tolerance:.2f}m)')
        
        if distance_to_start > self.start_tolerance:
            goal_handle.abort()
            result = FollowPath.Result()
            result.success = False
            result.message = f'Robot too far from start position: {distance_to_start:.2f}m'
            self.get_logger().error(result.message)
            return result
        
        self.get_logger().info('Starting path execution with Look-ahead Pure Pursuit...')
        self.get_logger().info(f'Look-ahead distance: {self.look_ahead_distance}m')
        
        # Main execution loop with Look-ahead logic
        rate = self.create_rate(self.update_rate)
        feedback_msg = FollowPath.Feedback()
        
        while True:
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowPath.Result()
                result.success = False
                result.message = 'Goal canceled by client'
                self.get_logger().info(result.message)
                return result
            
            # Get robot position
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            
            # Find closest point on path
            closest_index = self.find_closest_point_index(robot_x, robot_y)
            
            # Calculate distance to final goal
            final_waypoint = self.path_data[final_index]
            distance_to_final_goal = self.calculate_distance(
                robot_x, robot_y,
                final_waypoint['x'], final_waypoint['y']
            )
            
            # Log detailed progress
            self.get_logger().debug(
                f'[PROGRESS] Robot: ({robot_x:.2f}, {robot_y:.2f}) | '
                f'Closest WP: {closest_index}/{final_index} | '
                f'Distance to final: {distance_to_final_goal:.3f}m | '
                f'Goal tolerance: {self.goal_tolerance:.3f}m'
            )
            
            # Check if reached final goal (strict tolerance)
            if distance_to_final_goal < self.goal_tolerance:
                # Stop the robot before declaring success
                # 1. Publish velocity 0 (most important)
                stop_vel = Float64()
                stop_vel.data = 0.0
                self.target_velocity_pub.publish(stop_vel)
                
                # 2. Set target position to current position (make control error 0)
                stop_target = PoseStamped()
                stop_target.header.stamp = self.get_clock().now().to_msg()
                stop_target.header.frame_id = 'odom'
                stop_target.pose = self.current_pose
                self.target_point_pub.publish(stop_target)
                
                # 3. Declare action success
                goal_handle.succeed()
                result = FollowPath.Result()
                result.success = True
                result.message = f'Successfully reached final goal at ({final_waypoint["x"]:.2f}, {final_waypoint["y"]:.2f})'
                self.get_logger().info(result.message)
                self.get_logger().info(f'Final distance: {distance_to_final_goal:.4f}m < tolerance: {self.goal_tolerance:.3f}m')
                return result
            
            # Find look-ahead point
            target_x, target_y, target_yaw, target_velocity, target_index = self.find_look_ahead_point(
                robot_x, robot_y, closest_index
            )
            
            # Log look-ahead target
            self.get_logger().debug(
                f'[TARGET] Look-ahead point | '
                f'Index: {target_index} | '
                f'Position: ({target_x:.2f}, {target_y:.2f}) | '
                f'Velocity: {target_velocity:.2f} m/s'
            )
            
            # Apply velocity reversal in reverse mode
            if self.reverse_mode:
                target_velocity = -target_velocity
            
            # Update current index for feedback
            self.current_index = closest_index
            
            # Log progress every second (10 iterations at 10Hz)
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
            
            if self._log_counter % 10 == 0:
                self.get_logger().info(
                    f'[PROGRESS] Robot: ({robot_x:.2f}, {robot_y:.2f}) | '
                    f'Closest WP: {closest_index}/{final_index} | '
                    f'Target WP: {target_index} | '
                    f'Distance to final: {distance_to_final_goal:.3f}m'
                )
            
            # Publish target point (look-ahead point)
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = 'odom'
            target_msg.pose.position.x = target_x
            target_msg.pose.position.y = target_y
            target_msg.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            target_msg.pose.orientation.z = math.sin(target_yaw / 2.0)
            target_msg.pose.orientation.w = math.cos(target_yaw / 2.0)
            
            self.target_point_pub.publish(target_msg)
            
            # Publish target velocity
            velocity_msg = Float64()
            velocity_msg.data = target_velocity
            self.target_velocity_pub.publish(velocity_msg)
            
            # Update feedback
            total_waypoints = len(self.path_data)
            if self.reverse_mode:
                progress = (len(self.path_data) - 1 - closest_index) / (total_waypoints - 1)
            else:
                progress = closest_index / (total_waypoints - 1) if total_waypoints > 1 else 1.0
            
            feedback_msg.progress = float(progress)
            feedback_msg.distance_to_goal = float(distance_to_final_goal)
            feedback_msg.current_index = int(closest_index)
            goal_handle.publish_feedback(feedback_msg)
            
            # Log feedback every 1 second (10 loops at 10Hz)
            if not hasattr(self, '_feedback_counter'):
                self._feedback_counter = 0
            self._feedback_counter += 1
            if self._feedback_counter >= 10:
                self.get_logger().info(
                    f'Progress: {progress*100:.1f}% | '
                    f'Index: {closest_index}/{total_waypoints-1} | '
                    f'Distance: {distance_to_final_goal:.2f}m | '
                    f'Position: ({robot_x:.2f}, {robot_y:.2f})'
                )
                self._feedback_counter = 0
            
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = PathPlayerServer()
    # 멀티스레드 익스큐터 사용 (기본 4개 이상의 스레드 활용)
    executor = MultiThreadedExecutor()
    try:
        # 노드를 익스큐터에 추가하여 실행
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
