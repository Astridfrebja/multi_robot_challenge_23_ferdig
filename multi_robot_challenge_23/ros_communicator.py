#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ROSCommunicator:
    """
    ROS Communication - EN ansvar: Kun ROS messaging
    
    Single Responsibility: Kun ROS publisher/subscriber håndtering
    """
    
    def __init__(self, node_ref: Node):
        self.node = node_ref
        
        # Publishers
        self.big_fire_pub = None
        self.fire_position_pub = None
        self.fire_extinguished_pub = None
        
        # Subscribers
        self.big_fire_sub = None
        self.fire_position_sub = None
        self.fire_extinguished_sub = None
        
        self.node.get_logger().info('ROSCommunicator initialisert')

    def setup_big_fire_publishers(self):
        """Sett opp Big Fire publishers"""
        self.big_fire_pub = self.node.create_publisher(
            String, 'big_fire_detected', 10
        )
        self.fire_position_pub = self.node.create_publisher(
            String, 'robot_at_fire', 10
        )
        self.fire_extinguished_pub = self.node.create_publisher(
            String, 'fire_extinguished', 10
        )

    def setup_big_fire_subscribers(self, callbacks):
        """Sett opp Big Fire subscribers"""
        self.big_fire_sub = self.node.create_subscription(
            String, 'big_fire_detected', callbacks['big_fire'], 10
        )
        self.fire_position_sub = self.node.create_subscription(
            String, 'robot_at_fire', callbacks['robot_at_fire'], 10
        )
        self.fire_extinguished_sub = self.node.create_subscription(
            String, 'fire_extinguished', callbacks['fire_extinguished'], 10
        )

    def publish_big_fire_detection(self, position: tuple, robot_id: str):
        """Publiser Big Fire detection"""
        msg = String()
        msg.data = f"BIG_FIRE_DETECTED:{position[0]}:{position[1]}:{robot_id}"
        self.big_fire_pub.publish(msg)

    def publish_robot_at_fire(self, robot_id: str):
        """Publiser at robot er ved brannen"""
        msg = String()
        msg.data = f"{robot_id}:AT_FIRE"
        self.fire_position_pub.publish(msg)

    def publish_fire_extinguished(self):
        """Publiser at brannen er slukket"""
        msg = String()
        msg.data = "FIRE_EXTINGUISHED"
        self.fire_extinguished_pub.publish(msg)

    def publish_twist(self, linear_x: float, angular_z: float):
        """Publiser bevegelseskommandoer"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.node.cmd_vel_publisher.publish(twist_msg)
