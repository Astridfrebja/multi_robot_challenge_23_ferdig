#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import os

# Importerer refaktorert search & rescue coordinator
from .search_rescue_coordinator import SearchRescueCoordinator

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Opprett publisher og subscriber her
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # Identifiser robot basert på namespace
        self.robot_id = self.get_namespace().strip('/')
        self.get_logger().info(f'Robot identifisert som: {self.robot_id}')
        
        # Variabler for å lagre startposisjon
        self.start_position = None
        self.start_time = self.get_clock().now()

        # Initialiser refaktorert search & rescue coordinator
        self.navigation = SearchRescueCoordinator(self)
        
        self.get_logger().info(f'S&R ROBOT ({self.robot_id}): Wall Following + BUG2 + GoTo Goal + ArUco Detection + Big Fire Coordination!')


    def scan_callback(self, msg):
        """Håndterer laser scan data"""
        # Send LIDAR-data til coordinator
        self.navigation.process_scan(msg)

    def odom_callback(self, msg):
        """Håndterer odometri data"""
        # Lagre startposisjon ved første odometri-melding
        if self.start_position is None:
            self.start_position = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y
            }
            self.get_logger().info(f'Startposisjon lagret: x={self.start_position["x"]:.2f}, y={self.start_position["y"]:.2f}')
        
        # Send odometri-data til coordinator
        self.navigation.process_odom(msg)

    def publish_twist(self, linear_x, angular_z):
        """Hjelpefunksjon for å publisere bevegelseskommandoer"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_node = MyRobotNode()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()