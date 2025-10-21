#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower:
    """
    Wall Following klasse - EN ansvar: Wall following navigasjon
    
    Single Responsibility:  Kun wall following logikk
    """
    
    # Tilstandsdefinisjoner
    STATE_TURN_LEFT = 0      # Vegg foran, må snu unna
    STATE_FOLLOW_WALL = 1    # Kjør parallelt og korriger kurs
    STATE_TURN_RIGHT = 2     # Veggen har stoppet, må snu
    
    # --- INNSTILLINGER ---
    WALL_DISTANCE = 0.5      # Ønsket avstand fra veggen 
    FRONT_THRESHOLD = 0.8    # Reagere på veggen foran.
    KP_ANGULAR = 0.3         # P-kontroller for vinkelstyring
    LINEAR_SPEED = 0.3       # Konstant fremoverhastighet
    MAX_RANGE = 3.5          
    TURN_LEFT_SPEED_ANG = 0.8 # Svingehastighet for innvendige hjørner (økt)
    TURN_LEFT_SPEED_LIN = 0.0 # Ingen fremoverfart under sving
    
    def get_front_distance(self, scan: LaserScan) -> float:
        """Hjelpefunksjon for å hente avstand foran roboten."""
        return self._range_at_deg(scan, 0.0)
    
    def __init__(self, node_ref: Node, sensor_manager=None):
        self.node = node_ref
        self.sensor_manager = sensor_manager
        
        # Wall following state
        self.state = self.STATE_FOLLOW_WALL
        self.regions = {}
        
        # Timing for state transitions
        self.state_start_time = None
        self.turn_timeout = 3.0  # Maximum time to spend in TURN_LEFT
        
        # Setup publisher
        from geometry_msgs.msg import Twist
        self.cmd_vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        # State actions mapping
        self.state_actions = {
            self.STATE_TURN_LEFT: self.do_turn_left,
            self.STATE_FOLLOW_WALL: self.do_follow_wall,
            self.STATE_TURN_RIGHT: self.do_turn_right,
        }
        
        self.node.get_logger().info('🧱 WallFollower initialisert')

    def follow_wall(self, msg: LaserScan = None, target_direction=None):
        """
        Wall following logikk
        
        Args:
            msg: LaserScan data (optional, kan bruke sensor_manager)
            target_direction: (x, y) retning for guided wall following
        """
        # Bruk sensor_manager hvis tilgjengelig og ingen msg gitt
        if msg is None and self.sensor_manager:
            if not self.sensor_manager.is_scan_valid():
                self.stop_robot()
                return
            msg = self.sensor_manager.get_latest_scan()
        elif not msg or not msg.ranges:
            self.stop_robot()
            return

        # Process laser scan data
        self.process_laser_scan(msg)
        
        # Take action based on current state
        self.take_action()

    def process_laser_scan(self, msg: LaserScan):
        """Process laser scan and update regions"""
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = self.MAX_RANGE
        ranges[ranges == 0.0] = self.MAX_RANGE
        
        n = len(ranges)

        def safe_min(slice_array):
            return np.min(slice_array) if len(slice_array) > 0 else self.MAX_RANGE

        # Inndeling for veggfølging med 360 skanning
        self.regions = {
            'front': min(safe_min(ranges[0:int(n*0.05)]), safe_min(ranges[int(n*0.95):])),  
            'right': safe_min(ranges[int(n*0.80):int(n*0.90)]),  
            'back_right': safe_min(ranges[int(n*0.75):int(n*0.80)]),  
        }

    def decide_state(self):
        """Decide which state to use based on sensor readings"""
        d_front = self.regions.get('front', self.MAX_RANGE)
        d_right = self.regions.get('right', self.MAX_RANGE)
        d_back_right = self.regions.get('back_right', self.MAX_RANGE)
        
        # Check for timeout in TURN_LEFT state
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        if self.state == self.STATE_TURN_LEFT and self.state_start_time is not None:
            elapsed = current_time - self.state_start_time
            if elapsed > self.turn_timeout:
                self.node.get_logger().warn(f"🧱 TURN_LEFT timeout ({elapsed:.1f}s), forcing FOLLOW_WALL")
                return self.STATE_FOLLOW_WALL
        
        # Only log regions when state changes or every 10th call
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        
        if self._log_counter % 50 == 0:  # Log every 50th time
            self.node.get_logger().info(
                f"🧱 WallFollower regions: front={d_front:.2f}, right={d_right:.2f}, back_right={d_back_right:.2f}, current_state={self.state}"
            )
        
        # Vegg foran - må snu unna
        if d_front < self.FRONT_THRESHOLD:
            return self.STATE_TURN_LEFT
        
        # Veggen har stoppet, utvendig hjørne
        elif d_right > (self.WALL_DISTANCE + 0.4) and d_back_right > (self.WALL_DISTANCE + 0.4):
            return self.STATE_TURN_RIGHT
            
        # Følg veggen - mer fleksibel overgang fra TURN_LEFT
        else:
            # Hvis vi er i TURN_LEFT og front er nå fri, gå til FOLLOW_WALL
            if self.state == self.STATE_TURN_LEFT and d_front > self.FRONT_THRESHOLD:
                return self.STATE_FOLLOW_WALL
            # Ellers følg veggen
            return self.STATE_FOLLOW_WALL

    def do_turn_left(self):
        """Action for turning left when obstacle ahead"""
        twist_msg = Twist()
        twist_msg.linear.x = self.TURN_LEFT_SPEED_LIN
        twist_msg.angular.z = self.TURN_LEFT_SPEED_ANG 
        return twist_msg

    def do_turn_right(self):
        """Action for turning right when wall ends"""
        twist_msg = Twist()
        twist_msg.linear.x = self.LINEAR_SPEED * 0.5
        twist_msg.angular.z = -0.15 
        return twist_msg

    def do_follow_wall(self):
        """Action for following wall with distance control"""
        d_right = self.regions['right']
        error = d_right - self.WALL_DISTANCE
        angular_vel = -self.KP_ANGULAR * error
        
        twist_msg = Twist()
        twist_msg.angular.z = max(min(angular_vel, 0.5), -0.5) 
        twist_msg.linear.x = self.LINEAR_SPEED
        return twist_msg

    def take_action(self):
        """Take action based on current state"""
        new_state = self.decide_state()
        
        # Track state changes and timing
        if new_state != self.state:
            self.state = new_state
            self.state_start_time = self.node.get_clock().now().nanoseconds / 1e9
            self.node.get_logger().info(f"🧱 State change to: {self.state}")

        state_str = {0: "TURN_LEFT", 1: "FOLLOW_WALL", 2: "TURN_RIGHT"}[self.state]
        
        # Only log state when it changes
        if not hasattr(self, '_last_logged_state') or self._last_logged_state != self.state:
            self.node.get_logger().info(f"🧱 WallFollower State: {state_str}")
            self._last_logged_state = self.state

        # Get and execute the appropriate action function
        action_function = self.state_actions.get(self.state)
        if action_function:
            twist_msg = action_function()
        else:
            self.node.get_logger().warn(f"🧱 Ukjent tilstand: {self.state}. Stopper.")
            twist_msg = Twist()

        # Only log cmd_vel when it changes significantly
        if not hasattr(self, '_last_cmd_vel') or abs(twist_msg.linear.x - self._last_cmd_vel[0]) > 0.1 or abs(twist_msg.angular.z - self._last_cmd_vel[1]) > 0.1:
            self.node.get_logger().info(
                f"🧱 Cmd_vel: linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f}"
            )
            self._last_cmd_vel = (twist_msg.linear.x, twist_msg.angular.z)

        self.publish_twist(twist_msg.linear.x, twist_msg.angular.z)

    def _range_at_deg(self, scan: LaserScan, deg, default=100.0):
        """Hent avstand ved vinkel (grader) fra siste LIDAR"""
        # Bruk sensor_manager hvis tilgjengelig
        if self.sensor_manager:
            return self.sensor_manager.get_range_at_angle(deg, default)
        
        # Fallback til direkte scan håndtering
        if scan is None or not scan.ranges:
            return default
        
        # Matematikk for å finne indeksen i ranges-listen
        rad = math.radians(deg)
        idx = int(round((rad - scan.angle_min) / scan.angle_increment))
        
        if idx < 0 or idx >= len(scan.ranges):
            return default
        
        d = scan.ranges[idx]
        if d is None or math.isnan(d) or math.isinf(d) or d == 0.0:
            return default
            
        return float(d)

    def publish_twist(self, linear_x, angular_z):
        """Publiserer bevegelseskommandoer"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """Stopper robot bevegelse"""
        self.publish_twist(0.0, 0.0)