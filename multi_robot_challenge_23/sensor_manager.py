#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point



class SensorManager:
    """
    Sensor Manager - EN ansvar: Håndtering av alle robot sensorer

    Single Responsibility: Kun sensorhåndtering og dataprosessering
    """

    def __init__(self, node_ref: Node):
        self.node = node_ref

        # Sensor data storage
        self.latest_scan = None
        self.latest_odom = None
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0

        # ArUco detection data
        self.current_marker_id = None
        self.detected_markers = {}
        self._logged_markers = set()
       
        self.aruco_callback = None
        self.aruco_detection_radius = 2.0  # meter

        # Setup subscribers
        self.setup_sensor_subscribers()
        self.node.get_logger().info('SensorManager initialisert')


    def setup_sensor_subscribers(self):
        """Sett opp alle sensor subscribers"""
        # LIDAR subscriber
        self.scan_sub = self.node.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Odometry subscriber
        self.odom_sub = self.node.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # ArUco marker ID subscriber
        self.marker_id_sub = self.node.create_subscription(
            Int64, 'marker_id', self.marker_id_callback, 10
        )

        # ArUco marker pose subscriber 
        self.marker_pose_sub = self.node.create_subscription(
            Pose, 'marker_map_pose', self.marker_pose_callback, 10
        )

    def scan_callback(self, msg: LaserScan):
        """Håndterer LIDAR scan data"""
        self.latest_scan = msg
        self.node.get_logger().debug('LIDAR data mottatt')

    def odom_callback(self, msg: Odometry):
        """Håndterer odometry data"""
        self.latest_odom = msg

        # Oppdater robot posisjon
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        # Oppdater robot orientering (yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Yaw (z-akse rotasjon)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

        self.node.get_logger().debug(f'Odometry oppdatert: pos=({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}), yaw={self.robot_orientation:.2f}')

    def marker_id_callback(self, msg: Int64):
        """Håndterer ArUco marker ID detection"""
        marker_id = msg.data
        self.current_marker_id = marker_id
        if marker_id not in self._logged_markers:
            self.node.get_logger().debug(f'ArUco ID {marker_id} registrert (venter på nærmere posisjon)')

    def marker_pose_callback(self, msg: Pose):
        """Håndterer ArUco marker pose i map koordinater"""
        if self.current_marker_id is not None:
            position = (
                msg.position.x,
                msg.position.y,
                msg.position.z
            )

            distance = math.hypot(
                position[0] - self.robot_position[0],
                position[1] - self.robot_position[1]
            )
            if distance > self.aruco_detection_radius:
                self.node.get_logger().debug(
                    f'ArUco ID {self.current_marker_id} observert på {distance:.2f} m (grense {self.aruco_detection_radius:.2f} m)'
                )
            else:
                if self.current_marker_id not in self._logged_markers:
                    self.node.get_logger().info(
                        f'ArUco ID {self.current_marker_id} innenfor {self.aruco_detection_radius:.1f} m: '
                        f'x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}'
                    )
                    self._logged_markers.add(self.current_marker_id)

            # Lagre detektert merke
            self.detected_markers[self.current_marker_id] = (
                position[0], position[1], self.node.get_clock().now()
            )

            # Varsle koordinering hvis callback er satt (oppdaterer posisjon fortløpende)
            if self.aruco_callback is not None:
                try:
                    marker_id = int(self.current_marker_id)
                    self.aruco_callback(marker_id, (position[0], position[1]))
                except Exception as e:
                    self.node.get_logger().warn(f'ArUco callback feilet: {e}')

    def get_latest_scan(self) -> LaserScan:
        """Hent siste LIDAR scan"""
        return self.latest_scan

    def get_latest_odom(self) -> Odometry:
        """Hent siste odometry data"""
        return self.latest_odom

    def get_robot_position(self) -> tuple:
        """Hent robot posisjon"""
        return self.robot_position

    def get_robot_orientation(self) -> float:
        """Hent robot orientering"""
        return self.robot_orientation

    def get_robot_pose(self) -> tuple:
        """Hent robot pose (position, orientation)"""
        return self.robot_position, self.robot_orientation

    def get_detected_markers(self) -> dict:
        """Hent alle detekterte ArUco markers"""
        return self.detected_markers

    def has_marker(self, marker_id: int) -> bool:
        """Sjekk om marker er detektert"""
        return marker_id in self.detected_markers


    def get_range_at_angle(self, angle_deg: float, default: float = 100.0) -> float:
        """Hent avstand ved vinkel (grader) fra LIDAR"""
        if self.latest_scan is None or not self.latest_scan.ranges:
            return default

        rad = math.radians(angle_deg)
        idx = int(round((rad - self.latest_scan.angle_min) / self.latest_scan.angle_increment))

        if idx < 0 or idx >= len(self.latest_scan.ranges):
            return default

        d = self.latest_scan.ranges[idx]
        if d is None or math.isnan(d) or math.isinf(d) or d == 0.0:
            return default
        
        return float(d)

    def get_front_distance(self) -> float:
        """Hent avstand foran roboten"""
        return self.get_range_at_angle(0.0)

    def get_left_distance(self) -> float:
        """Hent avstand til venstre for roboten"""
        return self.get_range_at_angle(90.0)

    def get_right_distance(self) -> float:
        """Hent avstand til høyre for roboten"""
        return self.get_range_at_angle(-90.0)

    def get_front_left_distance(self) -> float:
        """Hent avstand foran-venstre for roboten"""
        return self.get_range_at_angle(45.0)

    def get_front_right_distance(self) -> float:
        """Hent avstand foran-høyre for roboten"""
        return self.get_range_at_angle(-45.0)

    def is_obstacle_ahead(self, threshold: float = 0.8) -> bool:
        """Sjekk om det er hindring foran"""
        return self.get_front_distance() < threshold

    def is_obstacle_left(self, threshold: float = 0.8) -> bool:
        """Sjekk om det er hindring til venstre"""
        return self.get_left_distance() < threshold

    def is_obstacle_right(self, threshold: float = 0.8) -> bool:
        """Sjekk om det er hindring til høyre"""
        return self.get_right_distance() < threshold

    def get_obstacle_direction(self, threshold: float = 0.8) -> str:
        """Hent retning til nærmeste hindring"""
        front = self.get_front_distance()
        left = self.get_left_distance()
        right = self.get_right_distance()

        if front < threshold:
            return "front"
        elif left < threshold:
            return "left"
        elif right < threshold:
            return "right"
        else:
            return "none"

    def calculate_distance_to_point(self, target_position: tuple) -> float:
        """Beregn avstand til et punkt"""
        dx = target_position[0] - self.robot_position[0]
        dy = target_position[1] - self.robot_position[1]
        return math.sqrt(dx*dx + dy*dy)

    def calculate_angle_to_point(self, target_position: tuple) -> float:
        """Beregn vinkel til et punkt"""
        dx = target_position[0] - self.robot_position[0]
        dy = target_position[1] - self.robot_position[1]
        return math.atan2(dy, dx)

    def calculate_heading_error(self, target_position: tuple) -> float:
        """Beregn heading error til et punkt"""
        desired_heading = self.calculate_angle_to_point(target_position)
        heading_error = desired_heading - self.robot_orientation
        return self.normalize_angle(heading_error)

    @staticmethod
    def to_point(position: tuple) -> Point:
        """Konverter et (x, y) par til geometry_msgs/Point"""
        point = Point()
        if position is None:
            return point
        point.x = float(position[0])
        point.y = float(position[1])
        point.z = 0.0
        return point

    def normalize_angle(self, angle: float) -> float:
        """Normaliser vinkel til [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_scan_valid(self) -> bool:
        """Sjekk om LIDAR scan er gyldig"""
        return self.latest_scan is not None and len(self.latest_scan.ranges) > 0

    def is_odom_valid(self) -> bool:
        """Sjekk om odometry data er gyldig"""
        return self.latest_odom is not None

    def get_sensor_status(self) -> dict:
        """Hent status for alle sensorer"""
        return {
            'lidar_valid': self.is_scan_valid(),
            'odom_valid': self.is_odom_valid(),
            'map_available': self.occupancy_grid_manager.is_map_available(),
            'robot_position': self.robot_position,
            'robot_orientation': self.robot_orientation,
            'detected_markers_count': len(self.detected_markers),
            'front_distance': self.get_front_distance(),
            'left_distance': self.get_left_distance(),
            'right_distance': self.get_right_distance()
        }

