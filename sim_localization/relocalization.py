#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped,StaticTransformBroadcaster
import math
import numpy as np
from enum import Enum
from scipy.optimize import fsolve



class CourtBoundary(Enum):
    LEFT = 0     # x=0
    RIGHT = 1    # x=court_length
    BOTTOM = 2   # y=0
    TOP = 3      # y=court_width
    UNKNOWN = 4

class LaserLocalization(Node):
    def __init__(self):
        super().__init__('laser_localization')
        
        # Parameters
        self.declare_parameter('court_length', 15.0)
        self.declare_parameter('court_width', 7.0)
        self.court_length = self.get_parameter('court_length').value
        self.court_width = self.get_parameter('court_width').value
        
        # State variables
        self.estimated_x = self.court_length*0.8
        self.estimated_y = self.court_width*0.1
        self.estimated_yaw = 0.
        self.initial_pose_received = True
        
        # Laser beam angles (relative to robot front)
        self.laser_angles = [0,np.pi*2/5,np.pi*2/5*2,np.pi*2/5*3,np.pi*2/5*4]  # Example angles
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/laser_scan', self.scan_callback, 10)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.estimated_x_arr = []
        self.estimated_y_arr = []
        
        self.get_logger().info("Laser localization node initialized")

    def initial_pose_callback(self, msg):
        """Handle initial pose estimate from RViz2"""
        self.estimated_x = msg.pose.pose.position.x
        self.estimated_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        # self.estimated_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.initial_pose_received = True
        self.get_logger().info(f"Initial pose set to: x={self.estimated_x:.2f}, y={self.estimated_y:.2f}, yaw={math.degrees(self.estimated_yaw):.2f}°")

    def calculate_boundary_angles(self, x, y):
        """Calculate angles to the four court corners"""
        angles = {
    'bottom_left': (math.atan2(0 - y, 0 - x) + 2 * math.pi) % (2 * math.pi),
    'bottom_right': (math.atan2(0 - y, self.court_length - x) + 2 * math.pi) % (2 * math.pi),
    'top_right': (math.atan2(self.court_width - y, self.court_length - x) + 2 * math.pi) % (2 * math.pi),
    'top_left': (math.atan2(self.court_width - y, 0 - x) + 2 * math.pi) % (2 * math.pi)
}
        return angles

    def determine_boundary(self, angle, boundary_angles):
        """Determine which boundary a laser beam is hitting"""
        angle = angle % (2 * math.pi)
        
        if angle >= boundary_angles['top_left'] and angle < boundary_angles['bottom_left']:
            return CourtBoundary.LEFT
        elif angle >= boundary_angles['bottom_left'] and angle < boundary_angles['bottom_right']:
            return CourtBoundary.BOTTOM
        elif angle >= boundary_angles['bottom_right'] or angle < boundary_angles['top_right']:
            return CourtBoundary.RIGHT
        elif (angle >= boundary_angles['top_right'] and angle <= boundary_angles['top_left']):
            return CourtBoundary.TOP
        else:
            return CourtBoundary.UNKNOWN

    def find_top_bottom_measurements(self,msg):
        boundary_angles = self.calculate_boundary_angles(self.estimated_x, self.estimated_y)
        print(boundary_angles)
        # Process each of the five laser beams
        # Separate TOP and BOTTOM measurements
        top_measurements = []  # (beam_index, distance)
        bottom_measurements = []  # (beam_index, distance)
        
        for i, beam_angle in enumerate(self.laser_angles):
            if i >= len(msg.ranges):
                continue
                
            # Get global angle (robot yaw + beam angle)
            global_angle = (self.estimated_yaw + beam_angle) % (2 * math.pi)
            distance = msg.ranges[i]
            
            if math.isinf(distance) or math.isnan(distance):
                continue
                
            # Determine boundary
            boundary = self.determine_boundary(global_angle, boundary_angles)
            
            if boundary == CourtBoundary.TOP:
                top_measurements.append((i, distance))
            elif boundary == CourtBoundary.BOTTOM:
                bottom_measurements.append((i, distance))
        
        self.get_logger().info(f"Found {len(top_measurements)} TOP and {len(bottom_measurements)} BOTTOM measurements")
        return top_measurements, bottom_measurements,boundary_angles
    def fill_estimated_arr(self,boundary_angles,msg):
        for i, beam_angle in enumerate(self.laser_angles):
            # Get global angle (robot yaw + beam angle)
            global_angle = self.estimated_yaw + beam_angle
            distance = msg.ranges[i]
            
            if math.isinf(distance) or math.isnan(distance):
                continue
            
            # Determine which boundary this beam is hitting
            boundary = self.determine_boundary(global_angle, boundary_angles)
            if boundary == CourtBoundary.UNKNOWN:
                continue
            # Calculate position based on measured distance
            if boundary == CourtBoundary.LEFT:
                new_x = -distance * math.cos(global_angle)
                self.estimated_x_arr.append(new_x)
                self.get_logger().info(f"{i}:left,new_x = {new_x:2f}")
            elif boundary == CourtBoundary.RIGHT:
                new_x = self.court_length - distance * math.cos(global_angle)
                self.estimated_x_arr.append(new_x)
                self.get_logger().info(f"{i}:right,new_x = {new_x:2f}")
            elif boundary == CourtBoundary.BOTTOM:
                new_y = abs(distance * math.sin(global_angle))
                self.estimated_y_arr.append(new_y)
                self.get_logger().info(f"{i}:bottom,new_y = {new_y:2f}")
            elif boundary == CourtBoundary.TOP:
                new_y = self.court_width - distance * math.sin(global_angle)
                self.estimated_y_arr.append(new_y)
                self.get_logger().info(f"{i}:top,new_y = {new_y:.2f}")
            else:
                continue
         # 1. Estimate yaw using TOP and BOTTOM measurements
    def update_estimated_position(self):
        # Update position estimate if we have valid measurements
        if self.estimated_x_arr and self.estimated_y_arr:
            # 添加用户选择机制
            if len(self.estimated_x_arr) > 1:
                x_std = np.std(self.estimated_x_arr)
                if x_std > 0.05:  # Threshold for x values
                    self.get_logger().warn(f'X values too different: {self.estimated_x_arr}')
                    # 随机选择一个值
                    selected_x = np.random.choice(self.estimated_x_arr)
                    self.get_logger().info(f'Using randomly selected x: {selected_x:.2f}')
                    self.estimated_x = max(0., min(self.court_length, selected_x))
                else:
                    avg_x = sum(self.estimated_x_arr) / len(self.estimated_x_arr)
                    self.estimated_x = max(0., min(self.court_length, avg_x))
            else:
                self.estimated_x = self.estimated_x_arr[0]
            
            if len(self.estimated_y_arr) > 1:
                y_std = np.std(self.estimated_y_arr)
                if y_std > 0.05:  # Threshold for y values
                    self.get_logger().warn(f'Y values too different: {self.estimated_y_arr}')
                    # 随机选择一个值
                    selected_y = np.random.choice(self.estimated_y_arr)
                    self.get_logger().info(f'Using randomly selected y: {selected_y:.2f}')
                    self.estimated_y = max(0., min(self.court_width, selected_y))
                else:
                    avg_y = sum(self.estimated_y_arr) / len(self.estimated_y_arr)
                    self.estimated_y = max(0., min(self.court_width, avg_y))
            else:
                self.estimated_y = self.estimated_y_arr[0]
            
            self.estimated_y_arr.clear()
            self.estimated_x_arr.clear()
            return True
        return False
    def scan_callback(self, msg):
        """Process laser scan data to estimate position"""
        if not self.initial_pose_received:
            return
        self.initial_pose_received=None
        for _ in range(2):
            top_measurements, bottom_measurements,boundary_angles = self.find_top_bottom_measurements(msg)
            self.estimate_yaw(top_measurements, bottom_measurements)
            self.fill_estimated_arr(boundary_angles,msg)
            if self.update_estimated_position():
                self.publish_tf()
                self.get_logger().info(f"Updated position: x={self.estimated_x:.2f}, y={self.estimated_y:.2f}")


    def estimate_yaw(self,top_measurements, bottom_measurements):
        if top_measurements and bottom_measurements:
            self.get_logger().info("Attempting to estimate yaw from TOP and BOTTOM measurements")
            estimated_yaw_found = False
            
            # Try up to 3 different combinations
            for attempt in range(3):
                # Randomly select one TOP and one BOTTOM measurement
                top_idx = np.random.randint(0, len(top_measurements))
                bottom_idx = np.random.randint(0, len(bottom_measurements))
                
                top_i, top_dist = top_measurements[top_idx]
                bottom_i, bottom_dist = bottom_measurements[bottom_idx]
                
                # Get the corresponding beam angles
                top_beam_angle = self.laser_angles[top_i]
                bottom_beam_angle = self.laser_angles[bottom_i]
                
                self.get_logger().info(f"Attempt {attempt+1}: Using TOP beam({np.degrees(top_beam_angle):.2f}, {top_dist:.2f}m) "
                                      f"and BOTTOM beam({np.degrees(bottom_beam_angle):.2f}, {bottom_dist:.2f}m)")
                
                # Define the equation to solve: |d1·sin(θ1+yaw)| + |d2·sin(θ2+yaw)| = court_width
                def equation(yaw_var):
                    term1 = abs(top_dist * math.sin(top_beam_angle + yaw_var[0]))
                    term2 = abs(bottom_dist * math.sin(bottom_beam_angle + yaw_var[0]))
                    return term1 + term2 - self.court_width
                
                # Try to solve with fsolve
                try:
                    # Use current estimated yaw as initial guess
                    initial_guess = [self.estimated_yaw]
                    yaw_solution = fsolve(equation, initial_guess, xtol=1e-3)[0]
                    
                    # Normalize yaw to [-π, π]
                    yaw_solution = (yaw_solution + math.pi) % (2 * math.pi) - math.pi
                    
                    # Check if solution is reasonable
                    if abs(yaw_solution - self.estimated_yaw) < math.pi/2:  # Less than 90° difference
                        self.estimated_yaw = yaw_solution
                        self.get_logger().info(f"Yaw updated! New yaw: {yaw_solution:.2f}")
                        estimated_yaw_found = True
                        break
                    else:
                        self.get_logger().warn(f"Solution too different: {math.degrees(yaw_solution):.2f}° vs current {math.degrees(self.estimated_yaw):.2f}°")
                except Exception as e:
                    self.get_logger().error(f"fsolve failed: {str(e)}")
            
            if not estimated_yaw_found:
                self.get_logger().error("Failed to estimate yaw after 3 attempts")
        else:
            self.get_logger().warn("Insufficient TOP/BOTTOM measurements for yaw estimation")
    def publish_tf(self):
        """Publish the transform from odom to map"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = '/map'
        t.child_frame_id = '/odom'
        
        t.transform.translation.x = self.estimated_x
        t.transform.translation.y = self.estimated_y
        t.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        cy = math.cos(self.estimated_yaw * 0.5)
        sy = math.sin(self.estimated_yaw * 0.5)
        cp = math.cos(0 * 0.5)  # pitch = 0
        sp = math.sin(0 * 0.5)
        cr = math.cos(0 * 0.5)   # roll = 0
        sr = math.sin(0 * 0.5)
        
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        
        self.static_tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LaserLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
