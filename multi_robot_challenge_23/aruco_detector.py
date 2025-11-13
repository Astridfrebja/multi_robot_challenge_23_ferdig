#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose

class ArUcoDetector:
    """
    ArUco marker detection - EN ansvar: ArUco marker håndtering
    
    Single Responsibility: Kun ArUco detection og rapportering
    """
    
    def __init__(self, node_ref: Node, callback=None, memory=None): 
        self.node = node_ref
        
        # ArUco tracking
        self.detected_markers = {}  # {marker_id: (x, y, timestamp)}
        self.current_marker_id = None
        
        self.aruco_callback = callback
        self.memory = memory
        
        self.node.get_logger().info('📊 ArUcoDetector initialisert')

    def process_detected_aruco(self, marker_id: int, position: tuple):
        """Ny inngangsfunksjon for å håndtere deteksjon én gang"""
        
        if not self.memory.is_aruco_processed(marker_id):
            self.node.get_logger().info(f'📊 ArUco ID {marker_id} oppdaget på {position} - Utfører callback.')
  
            self.memory.mark_aruco_processed(marker_id)
        
            if self.aruco_callback:
                self.aruco_callback(marker_id, position)
            else:
                self.node.get_logger().warn(f'📊 No aruco_callback set for marker_id={marker_id}')

    def report_aruco_marker(self, marker_id: int, position: tuple):
        """Rapporter ArUco-merke"""
        self.node.get_logger().info(f'📊 ArUco ID {marker_id} oppdaget på {position}')
        
        # Kaller callback hvis den er satt (for Big Fire koordinering)
        if self.aruco_callback:
            self.node.get_logger().info(f'📊 Calling aruco_callback for marker_id={marker_id}')
            self.aruco_callback(marker_id, position)
        else:
            self.node.get_logger().warn(f'📊 No aruco_callback set for marker_id={marker_id}')
        
        # Spesialhåndtering for Big Fire (ID 4)
        if marker_id == 4:
            self.node.get_logger().info('🔥 BIG FIRE OPPDAGET!')
            return marker_id, position
        
        self.node.get_logger().info(f'📊 RAPPORTERER: ArUco ID {marker_id} på posisjon {position}')
        return marker_id, position

    def get_detected_markers(self) -> dict:
        """Hent alle detekterte markers"""
        return self.detected_markers

    def has_marker(self, marker_id: int) -> bool:
        """Sjekk om marker er detektert"""
        return marker_id in self.detected_markers
