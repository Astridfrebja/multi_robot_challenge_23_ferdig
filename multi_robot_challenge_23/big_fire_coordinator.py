#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from rclpy.node import Node
from std_msgs.msg import String
from .robot_memory import RobotMemory

class BigFireCoordinator:
    """
    Big Fire koordinering - EN ansvar: Leder & Supporter logikk
    
    Single Responsibility: Kun Big Fire koordinering
    (Nye endringer: Bruker RobotMemory flagg for å redusere repeterende logging)
    """
    
    # States
    NORMAL = "NORMAL"
    LEDER_GOING_TO_FIRE = "LEDER_GOING_TO_FIRE"
    LEDER_WAITING = "LEDER_WAITING"
    SUPPORTER_GOING_TO_FIRE = "SUPPORTER_GOING_TO_FIRE"
    EXTINGUISHING = "EXTINGUISHING"
    
    # Roller
    LEDER = "LEDER"
    SUPPORTER = "SUPPORTER"
    
    def __init__(self, node_ref: Node, robot_memory: RobotMemory):
        self.node = node_ref
        self.robot_id = self.node.get_namespace().strip('/')
        
        self.memory = robot_memory
        
        self.setup_communication()
        
        self.node.get_logger().info(f'BigFireCoordinator ({self.robot_id}) initialisert')
        self.node.get_logger().info(f'Topics: /big_fire_detected, /robot_at_fire, /fire_extinguished')

    def setup_communication(self):
        """Sett opp kommunikasjon for Big Fire koordinering"""
        # Publisher for Big Fire detection 
        self.big_fire_pub = self.node.create_publisher(
            String, '/big_fire_detected', 10
        )
        
        # Subscriber for Big Fire detection 
        self.big_fire_sub = self.node.create_subscription(
            String, '/big_fire_detected', self.big_fire_callback, 10
        )
        
        # Publisher for robot position at fire 
        self.fire_position_pub = self.node.create_publisher(
            String, '/robot_at_fire', 10
        )
        
        # Subscriber for robot position at fire 
        self.fire_position_sub = self.node.create_subscription(
            String, '/robot_at_fire', self.robot_at_fire_callback, 10
        )
        
        # Publisher for fire extinguished 
        self.fire_extinguished_pub = self.node.create_publisher(
            String, '/fire_extinguished', 10
        )
        
        # Subscriber for fire extinguished
        self.fire_extinguished_sub = self.node.create_subscription(
            String, '/fire_extinguished', self.fire_extinguished_callback, 10
        )

    def detect_big_fire(self, position: tuple):
        """Leder oppdager Big Fire"""
        
        is_new_detection = not self.memory.big_fire_logged
        
        if is_new_detection:
            # Setter state kun hvis den er ny
            self.memory.set_big_fire_detected_by_me(position)
            
            # Publisering sendes kun ved ny deteksjon
            self.publish_big_fire_detection(position) 
            
            self.node.get_logger().info(f'LEDER: Big Fire oppdaget på {position}!')
            self.node.get_logger().info('LEDER: Starter koordinering og navigerer mot brannen.')
            
            self.memory.big_fire_logged = True
    def big_fire_callback(self, msg: String):
        """Supporter mottar Big Fire melding fra Leder. Logger KUN første gangen per hendelse."""
        if "BIG_FIRE_DETECTED" in msg.data:
            parts = msg.data.split(':')
            if len(parts) < 3: return 
            
            position = (float(parts[1]), float(parts[2]))
            scout_id = parts[3] if len(parts) > 3 else "unknown"

            if self.memory.big_fire_logged:
                return
            
            # Sett tilstand og rolle i minnet
            self.memory.set_big_fire_detected_by_other(position)
            
            # Logges kun første gang
            self.node.get_logger().info(f'SUPPORTER: Mottok Big Fire melding fra {scout_id}!')
            self.memory.big_fire_logged = True 

    def robot_at_fire_callback(self, msg: String):
        """Håndterer meldinger om at annen robot er ved/forlater brannen. Logger kun ved tilstandsskifte."""
        parts = msg.data.split(":")
        if len(parts) != 2:
            return

        robot_id, status = parts
        if robot_id == self.robot_id:
            return

        if status == "AT_FIRE":
            if not self.memory.other_robot_at_fire:
                self.memory.set_other_robot_at_fire(True)
                self.node.get_logger().info(f'Annen robot ({robot_id}) er ved brannen!')
        elif status == "LEFT_FIRE":
            if self.memory.other_robot_at_fire:
                self.memory.set_other_robot_at_fire(False)
                self.node.get_logger().info(f'Annen robot ({robot_id}) forlot brannen.')

    def fire_extinguished_callback(self, msg: String):
        """Håndterer meldinger om at brannen er slukket. Logger KUN ved tilstandsskifte."""
        if "FIRE_EXTINGUISHED" in msg.data:
            if not self.memory.fire_extinguished: # Logg kun ved tilstandsskifte
                self.memory.set_fire_extinguished(True)
                self.node.get_logger().info('BRANNEN ER SLUKKET!')

    def update_state(self, robot_position: tuple, robot_orientation: float):
        """Oppdater Big Fire tilstand basert på posisjon"""
        pass

    def get_target_position(self) -> tuple:
        """Hent målposisjon for navigasjon"""
        return self.memory.big_fire_position

    def should_handle_big_fire(self) -> bool:
        """Sjekk om vi skal håndtere Big Fire koordinering"""
        return self.memory.should_handle_big_fire()

    def is_leder_waiting(self) -> bool:
        """Sjekk om Leder venter på Supporter"""
        return self.memory.is_leder_waiting()

    def log_waiting_state(self):
        """Egen loggfunksjon for 'vente' tilstanden for å unngå støy"""
        if not self.memory.waiting_logged:
            self.node.get_logger().info('LEDER: Roboten er ved målet og venter på supporter.')
            self.memory.waiting_logged = True

    def is_extinguishing(self) -> bool:
        """Sjekk om vi slukker brannen"""
        return self.memory.is_extinguishing()

    def is_goal_reached(self) -> bool:
        """Sjekk om mål er nådd"""
        return self.memory.is_goal_reached()

    def publish_big_fire_detection(self, position: tuple):
        """Leder publiserer Big Fire detection. Logger KUN første gang."""
        msg = String()
        msg.data = f"BIG_FIRE_DETECTED:{position[0]}:{position[1]}:{self.robot_id}"
        self.big_fire_pub.publish(msg)
        
        # Logg at vi publiserer KUN hvis det er første gang for denne hendelsen
        if not self.memory.big_fire_logged:
            self.node.get_logger().info(f'LEDER: Publiserer Big Fire på {position}')


    def publish_robot_at_fire(self):
        """Leder publiserer at den er ved brannen. Logger KUN ved tilstandsskifte."""
        msg = String()
        msg.data = f"{self.robot_id}:AT_FIRE"
        self.fire_position_pub.publish(msg)
        
        if not self.memory.i_am_at_fire: 
            self.memory.set_i_am_at_fire(True)
            self.node.get_logger().info('LEDER: Publiserer at jeg er ved brannen!')

    def publish_robot_left_fire(self):
        """Publiser at vi ikke lenger er ved brannen."""
        if self.memory.i_am_at_fire:
            self.memory.set_i_am_at_fire(False)
        msg = String()
        msg.data = f"{self.robot_id}:LEFT_FIRE"
        self.fire_position_pub.publish(msg)

    def publish_fire_extinguished(self):
        """Publiserer at brannen er slukket. Logges alltid ved utløsning."""
        msg = String()
        msg.data = "FIRE_EXTINGUISHED"
        self.fire_extinguished_pub.publish(msg)
        
        self.memory.set_fire_extinguished(True)
        self.memory.transition_to_normal()
        
        self.node.get_logger().info('Brannen slukket! Returnerer til normal utforskning.')

    def reset(self):
        """Reset Big Fire koordinering"""
        self.memory.reset_big_fire_state()