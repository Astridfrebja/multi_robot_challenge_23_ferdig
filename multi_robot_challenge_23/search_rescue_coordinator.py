 #!/usr/bin/env python3

# -*- coding: utf-8 -*-


from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry


# Importer alle komponenter

from .wall_follower import WallFollower

from .goal_navigator import GoalNavigator

from .bug2_navigator import Bug2Navigator

from .big_fire_coordinator import BigFireCoordinator

from .aruco_detector import ArUcoDetector

from .robot_memory import RobotMemory

from .sensor_manager import SensorManager


class SearchRescueCoordinator:

    """

    Search & Rescue koordinator - koordinerer alle komponenter

    """

    

    def __init__(self, node_ref: Node):

        self.node = node_ref

        self.robot_id = self.node.get_namespace().strip('/')

        

        self.robot_position = (0.0, 0.0)

        self.robot_orientation = 0.0

        

        self.sensor_manager = SensorManager(node_ref)

        self.robot_memory = RobotMemory()

        self.big_fire_coordinator = BigFireCoordinator(node_ref, self.robot_memory)

        self.aruco_detector = ArUcoDetector(node_ref, self.handle_aruco_detection)

        # Koble sensorens ArUco callback direkte til koordinatorens handler
        # slik at roboten stopper uansett hvilket ArUco-merke som oppdages
        self.sensor_manager.aruco_callback = self.handle_aruco_detection

        

        self.wall_follower = WallFollower(node_ref, self.sensor_manager)

        self.goal_navigator = GoalNavigator(node_ref, self.sensor_manager)

        self.bug2_navigator = Bug2Navigator(node_ref, self.wall_follower, self.goal_navigator) 

        

        self.node.get_logger().info(f'🤖 SearchRescueCoordinator ({self.robot_id}) initialisert')


    def process_scan(self, msg: LaserScan):

        """

        Hovedfunksjon - koordinerer navigasjon. Kaller KUN ÉN navigasjonskontroller per syklus.

        """

        

        # Oppdater tilstanden (viktig å gjøre FØR navigasjon sjekkes)

        self.big_fire_coordinator.update_state(self.robot_position, self.robot_orientation)

        

        big_fire_active = self.big_fire_coordinator.should_handle_big_fire()

        

        if big_fire_active:

            self.node.get_logger().debug('🔥 BIG FIRE KOORDINERING AKTIV')

            

            # Hent målet for Big Fire navigasjon

            target = self.big_fire_coordinator.get_target_position()

            

            # KORRIGERT FEIL: Bruker den nye is_moving_to_fire metoden

            if target and self.robot_memory.is_moving_to_fire():

                

                self.node.get_logger().info(f'🔥 BUG2: Target={target}. State={self.robot_memory.big_fire_state}')

                

                # Sett målet i Bug2Navigator

                self.bug2_navigator.set_goal(target)

                

                # Utfør Bug2-navigasjon (den håndterer GO_TO_GOAL og WALL_FOLLOWING)

                goal_reached = self.bug2_navigator.navigate(msg)

                

                if goal_reached:

                    self.bug2_navigator.stop_robot()

                    self.node.get_logger().info('🔥 BUG2: Mål nådd! Oppdaterer Big Fire state.')

                    

                    # Oppdater Big Fire-tilstanden

                    if self.robot_memory.my_role == self.robot_memory.LEDER:

                        self.robot_memory.transition_to_leder_waiting()

                    else: # Supporter

                        self.robot_memory.transition_to_extinguishing()

            else:

                # Ingen bevegelse (Venter, slukker, eller nylig detektert)

                self.bug2_navigator.stop_robot()

                self.handle_big_fire_state_logic() 

                

        else:

            # Standard utforskning (Big Fire inaktiv)

            self.bug2_navigator.clear_goal() # Tømmer målet og stopper Bug2

            

            # Wall Follower fortsetter letingen

            self.wall_follower.follow_wall(msg) 


    def process_odom(self, msg: Odometry):

        """Oppdater robot posisjon og orientering"""

        

        self.robot_position = self.sensor_manager.get_robot_position()

        self.robot_orientation = self.sensor_manager.get_robot_orientation()

        

        self.robot_memory.update_robot_pose(self.robot_position, self.robot_orientation)

        self.bug2_navigator.update_robot_pose(self.robot_position, self.robot_orientation)


    def handle_big_fire_state_logic(self):

        """Håndterer KUN tilstandsoverganger og publisering."""

        coordinator = self.big_fire_coordinator

        current_state = coordinator.memory.big_fire_state

        

        # Merk: Siden koordinatoren og minneobjektet deler konstanter, kan vi bruke memory.KONSTANT

        

        if current_state == coordinator.memory.LEDER_WAITING:

            self.node.get_logger().info('🔥 LEDER: In LEDER_WAITING state!')

            if not coordinator.memory.i_am_at_fire:

                coordinator.publish_robot_at_fire()

            

            if coordinator.memory.other_robot_at_fire:

                coordinator.memory.transition_to_extinguishing()

                self.node.get_logger().info('🔥 LEDER: Supporter ankommet - begynner slukking!')

                

        elif current_state == coordinator.memory.EXTINGUISHING:

            self.node.get_logger().info('🔥 SLUKKING PÅGÅR!')

            if not coordinator.memory.fire_extinguished:

                coordinator.publish_fire_extinguished()

                self.node.get_logger().info('🔥 Brannen slukket! Roboter returnerer til normal utforskning.')

                coordinator.memory.transition_to_normal()

                

        elif current_state == coordinator.memory.NORMAL:

            # Sjekk om den mottok melding (supporter) eller nettopp detekterte (leder)

            if coordinator.memory.big_fire_detected_by_me:

                self.node.get_logger().info('🔥 LEDER: Jeg oppdaget Big Fire - starter navigasjon!')

                coordinator.memory.transition_to_leder_going_to_fire()

            elif coordinator.memory.big_fire_detected_by_other:

                self.node.get_logger().info('🔥 SUPPORTER: Mottok Big Fire melding - starter navigasjon!')

                coordinator.memory.transition_to_supporter_going_to_fire()


    def handle_aruco_detection(self, marker_id: int, position: tuple):

        """Håndterer ArUco marker detection"""

        # Only log once per marker to avoid spam
        if not hasattr(self, '_processed_aruco_markers'):
            self._processed_aruco_markers = set()
        
        marker_key = f"{marker_id}_{position[0]:.1f}_{position[1]:.1f}"
        if marker_key in self._processed_aruco_markers:
            return  # Already processed this marker at this position
        
        self._processed_aruco_markers.add(marker_key)

        self.bug2_navigator.stop_robot() 

        self.wall_follower.stop_robot()

        self.node.get_logger().info(f'🛑 ROBOT STOPPED! ArUco ID {marker_id} oppdaget på {position}')

        if marker_id == 4:  # Big Fire

            self.node.get_logger().info(f'🔥 BIG FIRE DETECTED! Calling detect_big_fire({position})')

            self.big_fire_coordinator.detect_big_fire(position)

            # Kaller update_state umiddelbart for å sette i gang navigasjonen i neste process_scan

            self.big_fire_coordinator.update_state(self.robot_position, self.robot_orientation)

        else:

            self.node.get_logger().info(f'📊 ArUco ID {marker_id} på {position} - Roboten stopper for scoring!') 