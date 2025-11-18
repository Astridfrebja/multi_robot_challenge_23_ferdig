#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class GoalNavigator:
    """
    Go-to-Goal navigator - EN ansvar: Navigasjon til spesifikke mål
    
    Single Responsibility: Kun mål-navigasjon logikk
    """
    
    # --- INNSTILLINGER ---
    FORWARD_SPEED = 0.3
    TURN_SPEED = 0.5
    THRESHOLD = 0.8
    GOAL_THRESHOLD = 0.5  # meters
    P_GAIN = 1.0  # P-kontroll for heading
    
    def __init__(self, node_ref: Node, sensor_manager=None):
        self.node = node_ref
        self.sensor_manager = sensor_manager
        self.target_position = None
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0
        self.navigation_active = False
        
        # Setup publisher
        from geometry_msgs.msg import Twist
        self.cmd_vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        self.node.get_logger().info('GoalNavigator initialisert')

    def set_goal(self, position: tuple):
        """Sett nytt mål for navigasjon"""
        self.target_position = position
        self.navigation_active = True
        self.node.get_logger().info(f'Nytt mål satt: {position}')

    def clear_goal(self):
        """Fjern aktivt mål"""
        self.target_position = None
        self.navigation_active = False
        self.node.get_logger().info('Mål fjernet')

    def update_robot_pose(self, position: tuple, orientation: float):
        """Oppdater robot posisjon og orientering"""
        self.robot_position = position
        self.robot_orientation = orientation
        
        # Oppdater også sensor_manager hvis tilgjengelig
        if self.sensor_manager:
            self.sensor_manager.robot_position = position
            self.sensor_manager.robot_orientation = orientation

    def navigate_to_goal(self, msg: LaserScan = None) -> bool:
        """
        Naviger mot mål med obstacle avoidance
        
        Args:
            msg: LaserScan data (optional, kan bruke sensor_manager)
        
        Returns:
            bool: True hvis mål er nådd
        """
        if not self.navigation_active or not self.target_position:
            return False

        # Sjekk om mål er nådd
        if self.is_goal_reached():
            self.node.get_logger().info('GOAL REACHED!')
            self.clear_goal()
            return True

        # Naviger mot mål
        self.go_to_goal_navigation(msg)
        return False

    def navigate_to_goal_keep_target(self, msg: LaserScan = None) -> bool:
        """
        Naviger mot mål med obstacle avoidance (beholder målet)
        
        Args:
            msg: LaserScan data (optional, kan bruke sensor_manager)
        
        Returns:
            bool: True hvis mål er nådd
        """
        if not self.navigation_active or not self.target_position:
            return False

        # Sjekk om mål er nådd
        if self.is_goal_reached():
            self.node.get_logger().info('GOAL REACHED! (beholder mål)')
            self.stop_robot()  
            return True

        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 1
        
        if self._debug_counter % 100 == 0:  
            distance = math.sqrt(
                (self.target_position[0] - self.robot_position[0])**2 +
                (self.target_position[1] - self.robot_position[1])**2
            )
            self.node.get_logger().info(f'Navigating to goal: {self.target_position} from {self.robot_position} (distance: {distance:.2f}m)')

        # Naviger mot mål
        self.go_to_goal_navigation(msg)
        return False

    def is_goal_reached(self) -> bool:
        """Sjekk om mål er nådd"""
        if not self.target_position:
            return False
        
        distance = math.sqrt(
            (self.target_position[0] - self.robot_position[0])**2 +
            (self.target_position[1] - self.robot_position[1])**2
        )
        
        if not hasattr(self, '_goal_check_counter'):
            self._goal_check_counter = 0
        self._goal_check_counter += 1
        
        if self._goal_check_counter % 50 == 0: 
            self.node.get_logger().info(f'Goal check: robot=({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}), target=({self.target_position[0]:.2f}, {self.target_position[1]:.2f}), distance={distance:.3f}m, threshold={self.GOAL_THRESHOLD}m')
        
        return distance <= self.GOAL_THRESHOLD

    def go_to_goal_navigation(self, msg: LaserScan = None):
        """Go-to-Goal navigasjon med obstacle avoidance"""
        if not self.target_position:
            return
        
        # Beregn retning til mål
        dx = self.target_position[0] - self.robot_position[0]
        dy = self.target_position[1] - self.robot_position[1]
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # Hent sensor data 
        if self.sensor_manager and self.sensor_manager.is_scan_valid():
            dL = self.sensor_manager.get_range_at_angle(+15.0)
            dR = self.sensor_manager.get_range_at_angle(-15.0)
            dF = self.sensor_manager.get_range_at_angle(0.0)
        elif msg:
            dL = self._range_at_deg(msg, +15.0)
            dR = self._range_at_deg(msg, -15.0)
            dF = self._range_at_deg(msg, 0.0)
        else:
            # Ingen sensor data tilgjengelig
            self.stop_robot()
            return
        
        # Enkel goal navigation logikk
        linear_x, angular_z = self.calculate_commands(dx, dy, distance_to_goal, dL, dR, dF)
        
        # Publiser kommandoer
        self.publish_twist(linear_x, angular_z)

    def calculate_commands(self, dx: float, dy: float, distance_to_goal: float, 
                          dL: float, dR: float, dF: float) -> tuple:
        """Beregn lineær og vinkelhastighet"""
        linear_x = 0.0
        angular_z = 0.0
        
        # Obstacle avoidance har høyest prioritet
        if dF < self.THRESHOLD:
            # Obstacle foran - sving til side med minst hinder
            if dL > dR:
                angular_z = self.TURN_SPEED  # Sving venstre
            else:
                angular_z = -self.TURN_SPEED  # Sving høyre
            linear_x = 0.1  # Sakte fremover
        else:
            # Ingen hinder foran - naviger mot mål
            desired_heading = math.atan2(dy, dx)
            heading_error = desired_heading - self.robot_orientation
            
            # Normaliser vinkel
            heading_error = self.normalize_angle(heading_error)
            
            # P-kontroll for heading
            angular_z = self.P_GAIN * heading_error
            
            # Adaptiv lineær hastighet basert på avstand til mål
            linear_x = self.calculate_adaptive_speed(distance_to_goal)
        
        return linear_x, angular_z

    def calculate_adaptive_speed(self, distance_to_goal: float) -> float:
        """Beregn adaptiv hastighet basert på avstand til mål"""
        if distance_to_goal > 2.0:
            return self.FORWARD_SPEED
        elif distance_to_goal > 1.0:
            return self.FORWARD_SPEED * 0.7
        else:
            return self.FORWARD_SPEED * 0.4

    def normalize_angle(self, angle: float) -> float:
        """Normaliser vinkel til [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _range_at_deg(self, scan: LaserScan, deg: float, default: float = 100.0) -> float:
        """Hent avstand ved vinkel (grader) fra LIDAR"""
        # Bruk sensor_manager hvis tilgjengelig
        if self.sensor_manager:
            return self.sensor_manager.get_range_at_angle(deg, default)
        
        # Fallback til direkte scan håndtering
        if scan is None or not scan.ranges:
            return default
        
        rad = math.radians(deg)
        idx = int(round((rad - scan.angle_min) / scan.angle_increment))
        
        if idx < 0 or idx >= len(scan.ranges):
            return default
        
        d = scan.ranges[idx]
        if d is None or math.isnan(d) or math.isinf(d) or d == 0.0:
            return default
            
        return float(d)

    def publish_twist(self, linear_x: float, angular_z: float):
        """Publiserer bevegelseskommandoer"""
        if not hasattr(self, '_last_published_cmd'):
            self._last_published_cmd = (0.0, 0.0)
        
        if abs(linear_x - self._last_published_cmd[0]) > 0.1 or abs(angular_z - self._last_published_cmd[1]) > 0.1:
            self.node.get_logger().info(f'Publishing cmd_vel: linear={linear_x:.2f}, angular={angular_z:.2f}')
            self._last_published_cmd = (linear_x, angular_z)
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """Stopper robot bevegelse"""
        self.publish_twist(0.0, 0.0)
