#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from scoring_interfaces.srv import SetMarkerPosition
from geometry_msgs.msg import Point

class ScoringClient:
    """
    Scoring Client - Rapporterer ArUco-markører til scoring-systemet
    
    Single Responsibility: Kun kommunikasjon med scoring service
    """
    
    def __init__(self, node_ref: Node):
        self.node = node_ref
        
        # Opprett service client
        self.client = self.node.create_client(
            SetMarkerPosition, 
            '/set_marker_position'
        )
        
        # Logg opprettelse
        self.node.get_logger().info('📊 ScoringClient initialisert')
        
        # Hold styr på hvilke markører som er rapportert
        self.reported_markers = set()
    
    def report_marker(self, marker_id: int, position: tuple) -> bool:
        """
        Rapporter ArUco-marker til scoring-systemet
        
        Args:
            marker_id: ArUco ID (0-4)
            position: (x, y) i map-koordinater
        
        Returns:
            True hvis akseptert, False ellers
        """
        # Unngå duplikater
        marker_key = f"{marker_id}_{position[0]:.1f}_{position[1]:.1f}"
        if marker_key in self.reported_markers:
            self.node.get_logger().debug(f'📊 Marker {marker_id} allerede rapportert')
            return True
        
        # Vent på at tjenesten er klar (med timeout)
        self.node.get_logger().info(f'📊 Venter på scoring service for marker {marker_id}...')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('❌ Scoring service ikke tilgjengelig!')
            return False
        
        # Lag forespørsel
        request = SetMarkerPosition.Request()
        request.marker_id = marker_id
        request.marker_position = Point()
        request.marker_position.x = float(position[0])
        request.marker_position.y = float(position[1])
        request.marker_position.z = 0.0
        
        self.node.get_logger().info(
            f'📊 Rapporterer ArUco ID {marker_id} på posisjon ({position[0]:.2f}, {position[1]:.2f})'
        )
        
        # Send forespørsel asynkront
        future = self.client.call_async(request)
        
        # Vent på svar (med timeout)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        
        if future.done():
            try:
                response = future.result()
                if response.accepted:
                    self.node.get_logger().info(f'✅ Scoring AKSEPTERT for marker {marker_id}!')
                    self.reported_markers.add(marker_key)
                    return True
                else:
                    self.node.get_logger().warn(f'⚠️ Scoring AVVIST for marker {marker_id}')
                    return False
            except Exception as e:
                self.node.get_logger().error(f'❌ Feil ved scoring: {e}')
                return False
        else:
            self.node.get_logger().error(f'❌ Timeout ved scoring av marker {marker_id}')
            return False
    
    def has_reported(self, marker_id: int, position: tuple) -> bool:
        """Sjekk om en marker allerede er rapportert"""
        marker_key = f"{marker_id}_{position[0]:.1f}_{position[1]:.1f}"
        return marker_key in self.reported_markers

