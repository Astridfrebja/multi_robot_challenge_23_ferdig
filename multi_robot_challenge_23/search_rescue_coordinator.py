#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import Optional

from geometry_msgs.msg import Twist

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String

# Importer alle komponenter
from .wall_follower import WallFollower
from .goal_navigator import GoalNavigator
from .bug2_navigator import Bug2Navigator
from .big_fire_coordinator import BigFireCoordinator
from .aruco_detector import ArUcoDetector
from .robot_memory import RobotMemory
from .sensor_manager import SensorManager
from .dfs_explorer import DfsExplorer
from .scoring_client import ScoringClient


class SearchRescueCoordinator:
    """
    Search & Rescue koordinator - koordinerer alle komponenter
    """

    # Kollisjons- / møtekonstanter
    AVOID_DISTANCE_TRIGGER = 0.75
    AVOID_DISTANCE_RELEASE = 1.40
    AVOID_FRONT_THRESHOLD = 0.70
    AVOID_FRONT_RELEASE = 1.10
    AVOID_TIMEOUT = 8.0
    AVOID_STALE_TIME = 3.0
    AVOID_HEADING_DIFFERENCE = 0.6  # rad (~35°)
    AVOID_BACK_DURATION = 0.8
    AVOID_HOLD_MIN = 1.0
    AVOID_ADVANCE_DURATION = 0.7

    HEADING_FRONT_THRESHOLD = math.radians(110.0)

    FIRE_SAME_SIDE_MAX_ANGLE = math.radians(120.0)
    HANDLED_FIRE_IGNORE_RADIUS = 1.5

    MARKER_CONFIRMATION_RADIUS = 2.0

    def __init__(self, node_ref: Node):
        self.node = node_ref
        self.robot_id = self.node.get_namespace().strip('/')

        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0

        self.sensor_manager = SensorManager(node_ref)
        self.robot_memory = RobotMemory()
        self.big_fire_coordinator = BigFireCoordinator(node_ref, self.robot_memory)
        self.aruco_detector = ArUcoDetector(node_ref, self.handle_aruco_detection)
        self.scoring_client = ScoringClient(node_ref)
        self.scoring_timer = self.node.create_timer(0.2, self.scoring_client.process_responses)
        self.marker_catalog = self._build_marker_catalog()

        self._processed_aruco_markers = set()
        self.pending_marker_reports = {}

        # Koble sensorens ArUco callback direkte til koordinatorens handler
        self.sensor_manager.aruco_callback = self.handle_aruco_detection

        try:
            robot_number = int(self.robot_id.rsplit('_', 1)[-1])
        except ValueError:
            robot_number = 0
        self.robot_number = robot_number

        follow_left = (robot_number % 2 == 1)
        self.wall_follower = WallFollower(node_ref, self.sensor_manager, follow_left=follow_left)
        side = 'venstre' if follow_left else 'høyre'
        self.node.get_logger().info(f'🧱 WallFollower konfig: følger {side} vegg')

        self.goal_navigator = GoalNavigator(node_ref, self.sensor_manager)
        self.bug2_navigator = Bug2Navigator(node_ref, self.wall_follower, self.goal_navigator)
        self.dfs_explorer = DfsExplorer()
        if hasattr(self.dfs_explorer, 'prefer_left'):
            self.dfs_explorer.prefer_left = follow_left

        # Del posisjon med andre roboter for trafikkregler
        self.robot_presence_pub = self.node.create_publisher(String, '/robot_presence', 10)
        self.robot_presence_sub = self.node.create_subscription(
            String, '/robot_presence', self.handle_robot_presence, 10
        )

        self.last_presence_pub_time = 0.0
        self.last_other_robot_update = None
        self.other_robot_id = None
        self.other_robot_number = None
        self.other_robot_yaw = None
        self.avoidance_active = False
        self.avoidance_release_time = 0.0
        self._avoidance_logged = False
        self.avoidance_mode = None
        self.avoidance_cooldown_until = 0.0
        self.handled_big_fires = set()
        self.handled_big_fire_positions = []
        self.active_big_fire_key = None
        self.active_marker_id = None
        self.active_marker_goal = None
        self.marker_navigation_start_time = 0.0
        self.avoidance_back_duration = self.AVOID_BACK_DURATION
        self.avoidance_hold_duration = self.AVOID_HOLD_MIN
        self.avoidance_return_duration = self.AVOID_ADVANCE_DURATION

        # --- ArUco sweep / scanning (for å oppdage vegg-merkene) ---
        self.last_aruco_check_position = (0.0, 0.0)
        self.last_aruco_check_time = 0.0
        self.aruco_check_interval = 30.0  # sekunder mellom sjekker
        self.aruco_check_distance = 4.0  # meter som må være flyttet før ny sjekk
        self.aruco_scan_angular_speed = 1.0  # rad/s for rotasjon
        self.aruco_half_turn_duration = (math.pi / 2.0) / self.aruco_scan_angular_speed
        self.aruco_scan_duration = self.aruco_half_turn_duration * 2.0 + 0.4
        self.is_doing_aruco_scan = False
        self.aruco_scan_start = 0.0
        self.pending_aruco_scan = False
        self._aruco_rotation_log = set()
        self.aruco_spin_margin = math.radians(5.0)

        # Marker-peek state (åpning)
        self.last_openings = set()
        self.marker_peek_state = None
        self.marker_peek_side = None
        self.marker_peek_start = 0.0
        self.marker_peek_rotation_speed = 0.75
        self.marker_peek_angle = math.radians(40.0)
        self.marker_peek_hold_duration = 0.6
        self.marker_peek_timeout = 3.0

        self.node.get_logger().info(f'🤖 SearchRescueCoordinator ({self.robot_id}) initialisert')

    def _build_marker_catalog(self) -> dict:
        """
        Returnerer en beskrivelse av ArUco-markører med tilhørende poeng.
        Verdier er basert på DAT160-spesifikasjonen og kan justeres ved behov.
        """
        return {
            0: {"label": "brannindikator", "points": 100},
            1: {"label": "brannindikator", "points": 100},
            2: {"label": "baby i fare", "points": 100},
            3: {"label": "brannindikator", "points": 100},
            4: {"label": "storbrann", "points": 100},
        }

    def _resume_normal_exploration(self):
        """
        Klargjør roboten for å gjenoppta ordinær utforskning etter en Big Fire-hendelse.
        """
        self.is_doing_aruco_scan = False
        self.pending_aruco_scan = False
        self.aruco_scan_start = 0.0
        self.last_aruco_check_position = self.robot_position
        self.last_aruco_check_time = time.time()

        self.dfs_explorer.reset()
        self.bug2_navigator.clear_goal()

        if hasattr(self.wall_follower, "state"):
            try:
                self.wall_follower.state = self.wall_follower.STATE_FOLLOW_WALL
                self.wall_follower.state_start_time = None
            except Exception:
                pass

        self.node.get_logger().info('🔁 Big Fire håndtert. DFS og veggfølging gjenopptas.')

    def _both_robots_within_fire_radius(self, radius: float = 2.0) -> bool:
        """True bare når begge roboter er nær nok OG på samme side (fri sikt til brannen)."""
        fire_pos = self.robot_memory.big_fire_position
        if fire_pos is None:
            return False

        if not (self.robot_memory.i_am_at_fire and self.robot_memory.other_robot_at_fire):
            return False

        my_distance = math.hypot(self.robot_position[0] - fire_pos[0],
                                 self.robot_position[1] - fire_pos[1])
        if my_distance > radius:
            return False

        other_pos = self.robot_memory.other_robot_position
        if other_pos is None:
            return False

        other_distance = math.hypot(other_pos[0] - fire_pos[0],
                                    other_pos[1] - fire_pos[1])
        if other_distance > radius:
            return False

        if not self._robots_same_side_of_fire(fire_pos, self.robot_position, other_pos):
            self.node.get_logger().info(
                '🔥 Begge roboter er nær brannen, men ser ut til å stå på hver sin side. Venter.'
            )
            return False

        return True

    # ---------------- ArUco sweep/hjelpefunksjoner ----------------

    def should_perform_aruco_scan(self) -> bool:
        """Return True dersom vi bør starte en ArUco-sveip."""
        if self.is_doing_aruco_scan:
            return False

        if self.pending_aruco_scan:
            if self.big_fire_coordinator.should_handle_big_fire() or self.avoidance_active:
                return False
            self.pending_aruco_scan = False
            return True

        if self.big_fire_coordinator.should_handle_big_fire() or self.avoidance_active:
            return False

        now = time.time()
        x, y = self.robot_position
        lx, ly = self.last_aruco_check_position
        dist = math.hypot(x - lx, y - ly)
        tdiff = now - self.last_aruco_check_time
        # Utfør sveip hvis roboten har flyttet seg mer enn 4 m ELLER det har gått 30s
        if dist < self.aruco_check_distance and tdiff < self.aruco_check_interval:
            return False


        if self.big_fire_coordinator.should_handle_big_fire() or self.avoidance_active:
            self.pending_aruco_scan = True
            return False

        return True

    def start_aruco_scan(self):
        """Start en periode hvor roboten roterer for å se etter ArUco-markører."""
        self.node.get_logger().info(f'👀 Starter ArUco-sveip (pos={self.robot_position})')
        self.is_doing_aruco_scan = True
        self.aruco_scan_start = time.time()
        self.pending_aruco_scan = False
        self._aruco_rotation_log = set()
        self.aruco_spin_speed = self.aruco_scan_angular_speed
        self.aruco_spin_direction = 1.0
        self.aruco_spin_accumulated = 0.0
        self.aruco_last_yaw = self.robot_orientation
        self._aruco_spin_log_state = None

        try:
            self.bug2_navigator.stop_robot()
        except Exception:
            pass
        try:
            self.wall_follower.stop_robot()
        except Exception:
            pass
        self._stop_all_motion()

    def perform_aruco_rotation(self):
        """Publiser en jevn 360° rotasjon for å dekke alle retninger."""
        current_yaw = self.robot_orientation
        if self.aruco_last_yaw is not None:
            delta_yaw = self._normalize_angle(current_yaw - self.aruco_last_yaw)
            self.aruco_spin_accumulated += abs(delta_yaw)
        self.aruco_last_yaw = current_yaw

        t = Twist()
        t.linear.x = 0.0
        t.angular.z = self.aruco_spin_speed * self.aruco_spin_direction

        if self.aruco_spin_accumulated >= (2 * math.pi) - self.aruco_spin_margin:
            self._stop_all_motion()
            self.finish_aruco_scan()
            return
        else:
            progress = min(self.aruco_spin_accumulated / (2 * math.pi), 0.999)
            stage = int(progress * 4)  # kvart-rundt-indikator
            if self._aruco_spin_log_state != stage:
                self.node.get_logger().info('👀 ArUco-sveip: roterer videre for dekning')
                self._aruco_spin_log_state = stage

        if not hasattr(self, '_aruco_cmd_pub'):
            self._aruco_cmd_pub = self.node.create_publisher(Twist, 'cmd_vel', 1)
        self._aruco_cmd_pub.publish(t)

    def finish_aruco_scan(self):
        """Avslutt sveip og oppdater tid/posisjon."""
        self.is_doing_aruco_scan = False
        self.last_aruco_check_position = self.robot_position
        self.last_aruco_check_time = time.time()
        self.pending_aruco_scan = False
        self._stop_all_motion()
        self.aruco_spin_accumulated = 0.0
        self.aruco_last_yaw = None
        self._aruco_spin_log_state = None
        self.node.get_logger().info('👁️ Ferdig ArUco-sveip — fortsetter normal navigasjon')

    def _cancel_aruco_scan(self, reason: str = None):
        """Avbryt pågående ArUco-sveip og nullstill state."""
        if not self.is_doing_aruco_scan:
            return
        self.is_doing_aruco_scan = False
        self.pending_aruco_scan = False
        self.aruco_spin_accumulated = 0.0
        self.aruco_last_yaw = None
        self._aruco_spin_log_state = None
        self._stop_all_motion()
        if reason:
            self.node.get_logger().info(f'👁️ ArUco-sveip avbrutt: {reason}')

    def _stop_all_motion(self):
        """Publiser null-hastighet på alle kontrollere."""
        try:
            self.bug2_navigator.stop_robot()
        except Exception:
            pass
        try:
            self.wall_follower.stop_robot()
        except Exception:
            pass
        try:
            self.goal_navigator.stop_robot()
        except Exception:
            pass
        if not hasattr(self, '_aruco_cmd_pub'):
            self._aruco_cmd_pub = self.node.create_publisher(Twist, 'cmd_vel', 1)
        self._aruco_cmd_pub.publish(Twist())

    def _detect_new_side_opening(self, openings) -> Optional[str]:
        """Returner første nye åpning mot venstre/høyre siden for peek."""
        current = set(openings or [])
        new_side = None
        for side in ('LEFT', 'RIGHT'):
            if side in current and side not in self.last_openings:
                new_side = side
                break
        self.last_openings = current
        return new_side

    def _can_start_marker_peek(self, side: Optional[str]) -> bool:
        if side is None:
            return False
        if self.marker_peek_state is not None:
            return False
        if self.is_doing_aruco_scan or self.pending_aruco_scan:
            return False
        if self.big_fire_coordinator.should_handle_big_fire():
            return False
        if self.dfs_explorer.has_active_goal() or self.dfs_explorer.has_pending_goals():
            return False
        if self.avoidance_active:
            return False
        return True

    def _start_marker_peek(self, side: str):
        """Initier kort markør-sjekk inn i ny sideåpning."""
        self.marker_peek_state = 'into'
        self.marker_peek_side = side
        self.marker_peek_start = time.time()
        direction = 'venstre' if side == 'LEFT' else 'høyre'
        self.node.get_logger().info(f'👀 Marker-peek: inspiserer åpning mot {direction}')
        self._stop_all_motion()
        angular = self.marker_peek_rotation_speed if side == 'LEFT' else -self.marker_peek_rotation_speed
        self._publish_marker_peek_twist(angular)

    def _update_marker_peek(self) -> bool:
        """Oppdater aktiv marker-peek. Returnerer True når prosesseringen skal avbrytes."""
        if self.marker_peek_state is None:
            return False

        now = time.time()
        elapsed = now - self.marker_peek_start
        rotate_duration = self.marker_peek_angle / max(self.marker_peek_rotation_speed, 1e-3)

        if elapsed > self.marker_peek_timeout:
            self.node.get_logger().warn('👀 Marker-peek: avbrutt (timeout).')
            self._finish_marker_peek()
            return True

        if self.marker_peek_state == 'into':
            if elapsed >= rotate_duration:
                self.marker_peek_state = 'hold'
                self.marker_peek_start = now
                self._stop_all_motion()
                self.node.get_logger().info('👀 Marker-peek: holder posisjon for markør-søk')
            else:
                angular = self.marker_peek_rotation_speed if self.marker_peek_side == 'LEFT' else -self.marker_peek_rotation_speed
                self._publish_marker_peek_twist(angular)
            return True

        if self.marker_peek_state == 'hold':
            if elapsed >= self.marker_peek_hold_duration:
                self.marker_peek_state = 'return'
                self.marker_peek_start = now
            else:
                self._stop_all_motion()
            return True

        if self.marker_peek_state == 'return':
            if elapsed >= rotate_duration:
                self._finish_marker_peek()
            else:
                angular = -self.marker_peek_rotation_speed if self.marker_peek_side == 'LEFT' else self.marker_peek_rotation_speed
                self._publish_marker_peek_twist(angular)
            return True

        return False

    def _publish_marker_peek_twist(self, angular_velocity: float):
        if not hasattr(self, '_aruco_cmd_pub'):
            self._aruco_cmd_pub = self.node.create_publisher(Twist, 'cmd_vel', 1)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_velocity
        self._aruco_cmd_pub.publish(twist)

    def _finish_marker_peek(self):
        direction = None
        if self.marker_peek_side == 'LEFT':
            direction = 'venstre'
        elif self.marker_peek_side == 'RIGHT':
            direction = 'høyre'
        self._stop_all_motion()
        if direction:
            self.node.get_logger().info(f'👀 Marker-peek fullført ({direction}). Fortsetter veggfølging.')
        else:
            self.node.get_logger().info('👀 Marker-peek fullført. Fortsetter veggfølging.')
        self.marker_peek_state = None
        self.marker_peek_side = None
        self.marker_peek_start = 0.0

    # -------------------------------------------------------------

    def process_scan(self, msg: LaserScan):
        """Hovedfunksjon - koordinerer navigasjon. Kaller KUN ÉN navigasjonskontroller per syklus."""
        # Oppdater Big Fire state tidlig
        self.big_fire_coordinator.update_state(self.robot_position, self.robot_orientation)
        big_fire_active = self.big_fire_coordinator.should_handle_big_fire()

        if big_fire_active:
            # Pause DFS mens storbrann håndteres
            if self.is_doing_aruco_scan:
                self._cancel_aruco_scan('Big Fire prioritet')
            self.dfs_explorer.clear_current_goal()

            # Hent målet for Big Fire navigasjon
            target = self.big_fire_coordinator.get_target_position()

            if target and self.robot_memory.is_moving_to_fire():
                robot_pos = self.robot_memory.robot_position
                distance_to_target = math.hypot(
                    target[0] - robot_pos[0],
                    target[1] - robot_pos[1]
                )

                threshold = self.bug2_navigator.GOAL_THRESHOLD + 0.15
                if distance_to_target <= threshold:
                    self.node.get_logger().info(
                        f'🔥 Allerede ved brannposisjon (d={distance_to_target:.2f} m). Stopper og oppdaterer tilstand.'
                    )
                    self.bug2_navigator.stop_robot()
                    if self.robot_memory.my_role == self.robot_memory.LEDER:
                        self.robot_memory.transition_to_leder_waiting()
                    else:
                        if not self.robot_memory.i_am_at_fire:
                            self.big_fire_coordinator.publish_robot_at_fire()
                            self.node.get_logger().info('🔥 SUPPORTER: Bekrefter ankomst ved brannen.')
                        self.robot_memory.transition_to_extinguishing()
                    self.handle_big_fire_state_logic()
                    return

                self.node.get_logger().info(f'🔥 BUG2: Target={target}. State={self.robot_memory.big_fire_state}')
                self.bug2_navigator.set_goal(target)
                goal_reached = self.bug2_navigator.navigate(msg)
                if goal_reached:
                    self.bug2_navigator.stop_robot()
                    self.node.get_logger().info('🔥 BUG2: Mål nådd! Oppdaterer Big Fire state.')
                    if self.robot_memory.my_role == self.robot_memory.LEDER:
                        self.robot_memory.transition_to_leder_waiting()
                    else:
                        if not self.robot_memory.i_am_at_fire:
                            self.big_fire_coordinator.publish_robot_at_fire()
                            self.node.get_logger().info('🔥 SUPPORTER: Bekrefter ankomst ved brannen.')
                        self.robot_memory.transition_to_extinguishing()
            else:
                # Ingen bevegelse 
                self.bug2_navigator.stop_robot()
                self.handle_big_fire_state_logic()

        else:
            # Normal utforskning / veggfølging / DFS

            if self.active_marker_id is not None:
                if self._handle_active_marker_navigation(msg):
                    return

            # Oppdater pose til DFS-modulen
            self.dfs_explorer.update_pose(self.robot_position, self.robot_orientation)

            # Oppdater LIDAR-data i wall follower før vi leser åpninger
            self.wall_follower.process_laser_scan(msg)

            openings = self.wall_follower.get_openings()
            new_side_opening = self._detect_new_side_opening(openings)

            if self.marker_peek_state:
                if self._update_marker_peek():
                    return
            elif self._can_start_marker_peek(new_side_opening):
                self._start_marker_peek(new_side_opening)
                return

            self.dfs_explorer.register_openings(openings)
            if self.dfs_explorer.has_pending_goals():
                self.node.get_logger().debug(
                    f'🧭 DFS: stakkstørrelse={self.dfs_explorer.pending_goal_count()} (aktivt mål={self.dfs_explorer.has_active_goal()})'
                )

            front_distance = self.wall_follower.regions.get('front', self.wall_follower.MAX_RANGE)
            if self.handle_robot_avoidance(front_distance):
                return

            # --- ArUco-sveip: hvis kriterier møtes, start / utfør / avslutt sveip ---
            if self.is_doing_aruco_scan:
                self.perform_aruco_rotation()
                return

            if self.should_perform_aruco_scan():
                self.start_aruco_scan()
                # publish initial rotation immediately
                self.perform_aruco_rotation()
                return


            # 1. Prioriter AKTIVT DFS-MÅL (Bug2 skal navigere)
            if self.dfs_explorer.has_active_goal():
                goal_reached = self.bug2_navigator.navigate(msg)
                
                if goal_reached or self.bug2_navigator.was_goal_aborted():
                    self.bug2_navigator.stop_robot()
                    self.bug2_navigator.clear_goal()
                    self.dfs_explorer.goal_reached()
                    if goal_reached:
                         self.node.get_logger().info('🧭 DFS: delmål nådd')
                    else:
                         self.node.get_logger().warn('🧭 DFS: delmål utilgjengelig. Hopper videre.')
                return # Fortsett å navigere eller prosesser neste syklus

            # 2. Hvis ikke aktivt mål, men HAR VENTENDE MÅL (hent det neste)
            if self.dfs_explorer.has_pending_goals():
                next_goal = self.dfs_explorer.next_goal()
                if next_goal:
                    self.node.get_logger().info(f'🧭 DFS: nytt delmål {next_goal}')
                    self.bug2_navigator.set_goal(next_goal)
                    # Gå til toppen av process_scan i neste syklus for å starte Bug2-navigasjon
                    return
            
            # 3. Hvis ingen aktive eller ventende mål, fortsett veggfølging (WallFollower)
            self.bug2_navigator.clear_goal()
            self.wall_follower.follow_wall(msg)

    # -------------------------------------------------------------

    def process_odom(self, msg: Odometry):
        """Oppdater robot posisjon og orientering"""
        self.robot_position = self.sensor_manager.get_robot_position()
        self.robot_orientation = self.sensor_manager.get_robot_orientation()

        self.robot_memory.update_robot_pose(self.robot_position, self.robot_orientation)
        self.bug2_navigator.update_robot_pose(self.robot_position, self.robot_orientation)
        self.publish_robot_presence()
        self._check_pending_marker_reports()

    def handle_big_fire_state_logic(self):
        """Håndterer KUN tilstandsoverganger og publisering."""
        coordinator = self.big_fire_coordinator
        current_state = coordinator.memory.big_fire_state

        if current_state == coordinator.memory.LEDER_WAITING:
            self.node.get_logger().info('🔥 LEDER: In LEDER_WAITING state!')
            if not coordinator.memory.i_am_at_fire:
                coordinator.publish_robot_at_fire()
            if coordinator.memory.other_robot_at_fire:
                coordinator.memory.transition_to_extinguishing()
                self.node.get_logger().info('🔥 LEDER: Supporter ankommet - begynner slukking!')

        elif current_state == coordinator.memory.EXTINGUISHING:
            self.node.get_logger().info('🔥 SLUKKING PÅGÅR!')
            if coordinator.memory.fire_extinguished:
                if coordinator.memory.big_fire_state != coordinator.memory.NORMAL:
                    coordinator.memory.transition_to_normal()
                self._mark_fire_handled(coordinator.memory.big_fire_position)
                self._resume_normal_exploration()
            else:
                if (
                    self.robot_memory.my_role == self.robot_memory.LEDER
                    and self._both_robots_within_fire_radius()
                ):
                    self._mark_fire_handled(coordinator.memory.big_fire_position)
                    coordinator.publish_fire_extinguished()
                    self._resume_normal_exploration()

        elif current_state == coordinator.memory.NORMAL:
            if coordinator.memory.big_fire_detected_by_me:
                self.node.get_logger().info('🔥 LEDER: Jeg oppdaget Big Fire - starter navigasjon!')
                coordinator.memory.transition_to_leder_going_to_fire()
            elif coordinator.memory.big_fire_detected_by_other:
                self.node.get_logger().info('🔥 SUPPORTER: Mottok Big Fire melding - starter navigasjon!')
                coordinator.memory.transition_to_supporter_going_to_fire()

    def handle_aruco_detection(self, marker_id: int, position: tuple):
        """Håndter ArUco-detektering med krav om nærhet før rapportering."""
        if marker_id in self._processed_aruco_markers:
            return

        marker_info = self.marker_catalog.get(marker_id, {
            "label": "ukjent objekt",
            "points": 0,
        })
        distance = self._distance_to_position(position)

        self.node.get_logger().info(
            f'🎯 ArUco merke ID {marker_id} ({marker_info["label"]}) observert på '
            f'({position[0]:.2f}, {position[1]:.2f}) – avstand {distance:.2f} m'
        )

        if distance > self.MARKER_CONFIRMATION_RADIUS:
            self._register_pending_marker(marker_id, position, marker_info, distance)
            return

        self._finalize_marker_detection(marker_id, position, marker_info, distance)

    def _register_pending_marker(self, marker_id: int, position: tuple, marker_info: dict, distance: float):
        """Lagre markør som må bekreftes når roboten er nærmere."""
        existing = self.pending_marker_reports.get(marker_id)
        self.pending_marker_reports[marker_id] = {
            'position': position,
            'label': marker_info.get("label", "ukjent objekt"),
        }

        if existing is None:
            if marker_id == 4:
                self.node.get_logger().info(
                    f'🔥 Big Fire registrert {distance:.2f} m unna (> {self.MARKER_CONFIRMATION_RADIUS:.1f} m). '
                    'Fortsetter nærmere før stopp og rapportering.'
                )
            else:
                self.node.get_logger().info(
                    f'🚶 Markør ID {marker_id} er {distance:.2f} m unna (> {self.MARKER_CONFIRMATION_RADIUS:.1f} m). '
                    'Venter med poengregistrering til roboten er innenfor 2.0 m.'
                )

        if marker_id == 4:
            self._handle_big_fire_detection(
                position,
                distance,
                within_radius=False,
                first_detection=(existing is None),
            )
        else:
            self._start_marker_navigation(marker_id, position)

    def _finalize_marker_detection(self, marker_id: int, position: tuple, marker_info: dict, distance: float):
        """Rapporter markøren når roboten er innenfor ønsket radius."""
        was_pending = marker_id in self.pending_marker_reports
        self.pending_marker_reports.pop(marker_id, None)

        if marker_id in self._processed_aruco_markers:
            return

        self._processed_aruco_markers.add(marker_id)

        self.node.get_logger().info(
            f'✅ ArUco ID {marker_id} ({marker_info["label"]}) er {distance:.2f} m unna '
            f'(<={self.MARKER_CONFIRMATION_RADIUS:.1f} m). Rapporterer til scoring-systemet.'
        )

        report_result = self.scoring_client.report_marker(marker_id, position)
        if report_result is False:
            self.node.get_logger().warn(
                f'⚠️ Klarte ikke å rapportere ArUco ID {marker_id} til scoringstjenesten.'
            )

        if marker_id == 4:
            self._handle_big_fire_detection(
                position,
                distance,
                within_radius=True,
                first_detection=not was_pending,
            )
        else:
            self._clear_active_marker_navigation(marker_id)

    def _distance_to_position(self, position: tuple) -> float:
        """Beregn avstand fra roboten til gitt posisjon."""
        if position is None:
            return float('inf')
        return math.hypot(position[0] - self.robot_position[0], position[1] - self.robot_position[1])

    def _check_pending_marker_reports(self):
        """Se om ventende markører nå er innenfor rapporteringsradius."""
        if not self.pending_marker_reports:
            return

        for marker_id, data in list(self.pending_marker_reports.items()):
            position = data.get('position')
            if position is None:
                continue

            distance = self._distance_to_position(position)
            if distance <= self.MARKER_CONFIRMATION_RADIUS:
                marker_info = self.marker_catalog.get(marker_id, {
                    "label": "ukjent objekt",
                    "points": 0,
                })
                self._finalize_marker_detection(marker_id, position, marker_info, distance)
            elif marker_id == 4:
                self._handle_big_fire_detection(
                    position,
                    distance,
                    within_radius=False,
                    first_detection=False,
                )

        if self.active_marker_id is None:
            self._start_next_pending_marker()

    def _handle_big_fire_detection(
        self,
        position: tuple,
        distance: float,
        within_radius: bool,
        first_detection: bool = False,
    ):
        """Koordiner spesiallogikk for Big Fire-markøren."""
        big_fire_key = self._big_fire_key(position)

        if self._is_fire_already_handled(position):
            self.node.get_logger().info('🔥 Big Fire på denne posisjonen er allerede slukket. Ignorerer.')
            return

        if big_fire_key in self.handled_big_fires:
            self.node.get_logger().info('🔥 Big Fire på denne posisjonen er allerede slukket. Ignorerer.')
            return

        self.active_big_fire_key = big_fire_key

        if first_detection:
            self.node.get_logger().info(
                f'🔥 BIG FIRE oppdaget {distance:.2f} m unna. Aktiverer koordinator.'
            )

        self._cancel_aruco_scan('Big Fire funnet')

        if first_detection or not self.big_fire_coordinator.should_handle_big_fire():
            self.big_fire_coordinator.detect_big_fire(position)

        self.big_fire_coordinator.update_state(self.robot_position, self.robot_orientation)

        if within_radius:
            if self.robot_memory.my_role == self.robot_memory.LEDER:
                self.node.get_logger().info(
                    f'🛑 Leder ved Big Fire (avstand {distance:.2f} m). Stopper roboten og avventer samarbeid.'
                )
                self.bug2_navigator.stop_robot()
                self.wall_follower.stop_robot()
                if self.robot_memory.big_fire_state != self.robot_memory.LEDER_WAITING:
                    self.robot_memory.transition_to_leder_waiting()
                if not self.robot_memory.i_am_at_fire:
                    self.big_fire_coordinator.publish_robot_at_fire()
            else:
                self.node.get_logger().info(
                    f'🔥 Supporter innenfor {self.MARKER_CONFIRMATION_RADIUS:.1f} m av Big Fire (avstand {distance:.2f} m).'
                )
                if not self.robot_memory.i_am_at_fire:
                    self.big_fire_coordinator.publish_robot_at_fire()
        else:
            if first_detection:
                self.node.get_logger().info(
                    f'🔥 Fortsetter mot Big Fire til avstand er <= {self.MARKER_CONFIRMATION_RADIUS:.1f} m.'
                )

    # --- Trafikkregler mellom roboter ---
    def publish_robot_presence(self):
        """Publiser egen posisjon periodisk slik at andre roboter kan planlegge."""
        now = self.get_time_seconds()
        if now - self.last_presence_pub_time < 0.4:
            return

        msg = String()
        msg.data = (
            f"POSE:{self.robot_id}:"
            f"{self.robot_position[0]:.3f}:{self.robot_position[1]:.3f}:"
            f"{self.robot_orientation:.3f}:{self.robot_number}"
        )
        self.robot_presence_pub.publish(msg)
        self.last_presence_pub_time = now

    def handle_robot_presence(self, msg: String):
        """Motta posisjonsinfo fra andre roboter."""
        try:
            parts = msg.data.split(':')
            if len(parts) != 6 or parts[0] != "POSE":
                return
            sender_id = parts[1]
            if sender_id == self.robot_id:
                return
            x = float(parts[2])
            y = float(parts[3])
            yaw = float(parts[4])
            number = int(parts[5])
        except (ValueError, IndexError):
            return

        self.robot_memory.other_robot_position = (x, y)
        self.other_robot_id = sender_id
        self.other_robot_number = number
        self.other_robot_yaw = yaw
        self.last_other_robot_update = self.get_time_seconds()

    def handle_robot_avoidance(self, front_distance: float) -> bool:
        """
        Enkle trafikkregler: lavest robot-ID får forkjørsrett.
        Returnerer True når vi skal pause annen logikk i denne syklusen.
        """
        now = self.get_time_seconds()

        if now < self.avoidance_cooldown_until:
            return False

        if (
            self.robot_memory.other_robot_position is None
            or self.last_other_robot_update is None
            or (now - self.last_other_robot_update) > self.AVOID_STALE_TIME
        ):
            if self.avoidance_active:
                self._reset_avoidance_state()
                self.node.get_logger().info('🤝 Ingen annen robot i nærheten. Fortsetter normal drift.')
            return False

        other_x, other_y = self.robot_memory.other_robot_position
        distance = math.hypot(other_x - self.robot_position[0], other_y - self.robot_position[1])
        bearing = math.atan2(other_y - self.robot_position[1], other_x - self.robot_position[0])
        heading_diff = abs(self._normalize_angle(bearing - self.robot_orientation))

        if self.avoidance_active:
            encounter_running = False
            try:
                if hasattr(self.wall_follower, 'update_robot_encounter'):
                    encounter_running = self.wall_follower.update_robot_encounter()
                elif hasattr(self.wall_follower, 'is_encounter_running'):
                    encounter_running = self.wall_follower.is_encounter_running()
            except Exception as exc:
                encounter_running = False
                self.node.get_logger().warn(f'🤝 Encounter update feilet: {exc}')

            if not encounter_running or now >= self.avoidance_release_time:
                self._reset_avoidance_state(cooldown=0.5)
                return False
            return True

        if distance > self.AVOID_DISTANCE_TRIGGER or front_distance > self.AVOID_FRONT_THRESHOLD:
            self._avoidance_logged = False
            return False

        if heading_diff > self.HEADING_FRONT_THRESHOLD:
            return False

        my_number = self.robot_number if self.robot_number is not None else 0
        other_number = self.other_robot_number if self.other_robot_number is not None else 999

        if self.avoidance_active:
            return True

        mode = 'priority' if my_number <= other_number else 'yield'
        label = self.other_robot_id or 'ukjent'
        if mode == 'priority':
            self.node.get_logger().info(
                f'🤝 Møter {label}. Har forkjørsrett, men rygger litt tilbake for å åpne passasje.'
            )
        else:
            self.node.get_logger().info(
                f'🤝 Gir forkjørsrett til {label}. Rygger og venter før jeg fortsetter.'
            )

        self.wall_follower.start_robot_encounter(mode)
        self.bug2_navigator.stop_robot()
        self._avoidance_logged = True
        self.avoidance_active = True
        encounter_timeout = getattr(self.wall_follower, 'ENCOUNTER_MAX_DURATION', self.AVOID_TIMEOUT)
        self.avoidance_release_time = now + max(self.AVOID_TIMEOUT, encounter_timeout)
        self.avoidance_mode = mode
        return True

    def get_time_seconds(self) -> float:
        """Hent ROS-klokkens tid i sekunder."""
        return self.node.get_clock().now().nanoseconds / 1e9

    def _heading_difference_ok(self, mode: str) -> bool:
        """Sjekk om forskjellen i heading tilsier at vi kan slippe videre."""
        if self.other_robot_yaw is None:
            return True
        diff = abs(self._normalize_angle(self.robot_orientation - self.other_robot_yaw))
        if mode == 'yield':
            return diff > self.AVOID_HEADING_DIFFERENCE
        return True

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normaliser vinkel til [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _start_marker_navigation(self, marker_id: int, position: Optional[tuple]):
        """Start målrettet navigasjon til et ArUco-merke (ikke Big Fire)."""
        if marker_id == 4:
            return
        if position is None:
            return
        if marker_id in self._processed_aruco_markers:
            return
        if self.big_fire_coordinator.should_handle_big_fire():
            return

        if self.active_marker_id is not None:
            if self.active_marker_id == marker_id:
                self.active_marker_goal = position
                try:
                    self.bug2_navigator.set_goal(position)
                except Exception:
                    pass
            return

        self.active_marker_id = marker_id
        self.active_marker_goal = position
        self.marker_navigation_start_time = self.get_time_seconds()

        self.node.get_logger().info(
            f'🎯 Navigerer mot ArUco ID {marker_id} for bekreftelse (mål={position}).'
        )

        try:
            self.dfs_explorer.clear_current_goal()
        except Exception:
            pass
        try:
            self.wall_follower.stop_robot()
        except Exception:
            pass
        try:
            self.bug2_navigator.clear_goal()
        except Exception:
            pass
        try:
            self.bug2_navigator.set_goal(position)
        except Exception:
            self.node.get_logger().warn('🎯 Klarte ikke å sette BUG2-mål for ArUco-navigasjon.')

    def _handle_active_marker_navigation(self, scan: LaserScan) -> bool:
        """Fortsett navigasjon mot aktivt ArUco-merke. Returnerer True hvis håndtert."""
        if self.active_marker_id is None or self.active_marker_goal is None:
            return False

        try:
            self.bug2_navigator.set_goal(self.active_marker_goal)
        except Exception:
            pass

        goal_reached = False
        try:
            goal_reached = self.bug2_navigator.navigate(scan)
        except Exception as exc:
            self.node.get_logger().warn(f'🎯 BUG2-feil under ArUco-navigasjon: {exc}')

        aborted = False
        try:
            aborted = self.bug2_navigator.was_goal_aborted()
        except Exception:
            aborted = False

        if aborted:
            self.node.get_logger().warn(
                f'🎯 ArUco ID {self.active_marker_id}: BUG2-abort. Prøver samme mål på nytt.'
            )
            try:
                self.bug2_navigator.clear_goal()
                self.bug2_navigator.set_goal(self.active_marker_goal)
            except Exception:
                pass
            return True

        distance = self._distance_to_position(self.active_marker_goal)
        if goal_reached or (distance is not None and distance <= self.MARKER_CONFIRMATION_RADIUS):
            try:
                self.bug2_navigator.stop_robot()
            except Exception:
                pass
            marker_id = self.active_marker_id
            if marker_id not in self._processed_aruco_markers:
                marker_info = self.marker_catalog.get(marker_id, {
                    "label": "ukjent objekt",
                    "points": 0,
                })
                self._finalize_marker_detection(
                    marker_id,
                    self.active_marker_goal,
                    marker_info,
                    distance if distance is not None else 0.0,
                )
        return True

    def _clear_active_marker_navigation(self, marker_id: Optional[int] = None):
        """Avslutt navigasjon mot aktivt ArUco-merke og gjenoppta normal modus."""
        if self.active_marker_id is None:
            return
        if marker_id is not None and marker_id != self.active_marker_id:
            return

        try:
            self.bug2_navigator.clear_goal()
        except Exception:
            pass
        self.active_marker_id = None
        self.active_marker_goal = None
        self.marker_navigation_start_time = 0.0
        self._start_next_pending_marker()

    def _start_next_pending_marker(self):
        """Start navigasjon mot neste ventende ArUco-merke dersom tilgjengelig."""
        if self.active_marker_id is not None:
            return
        if self.big_fire_coordinator.should_handle_big_fire():
            return

        for marker_id, data in list(self.pending_marker_reports.items()):
            if marker_id == 4:
                continue
            if marker_id in self._processed_aruco_markers:
                continue
            position = data.get('position')
            if position is None:
                continue
            self._start_marker_navigation(marker_id, position)
            break

    def _mark_fire_handled(self, position: tuple):
        """Registrer at en Big Fire er håndtert slik at vi ignorerer fremtidige observasjoner."""
        if position is None:
            return
        key = self._big_fire_key(position)
        if key not in self.handled_big_fires:
            self.handled_big_fires.add(key)
        if not any(
            math.hypot(position[0] - existing[0], position[1] - existing[1]) <= self.HANDLED_FIRE_IGNORE_RADIUS / 2.0
            for existing in self.handled_big_fire_positions
        ):
            self.handled_big_fire_positions.append((position[0], position[1]))
        self.active_big_fire_key = None

    def _is_fire_already_handled(self, position: tuple) -> bool:
        """Returner True dersom vi har håndtert en brann nær denne posisjonen tidligere."""
        if position is None:
            return False
        for handled in self.handled_big_fire_positions:
            if math.hypot(position[0] - handled[0], position[1] - handled[1]) <= self.HANDLED_FIRE_IGNORE_RADIUS:
                return True
        return False

    def _robots_same_side_of_fire(self, fire_pos: tuple, my_pos: tuple, other_pos: tuple) -> bool:
        """
        Sjekk om begge robotene står på omtrent samme side av brannen.
        Bruk vektoren fra brannen til hver robot og sørg for at vinkelen mellom dem er moderat.
        """
        vx1 = my_pos[0] - fire_pos[0]
        vy1 = my_pos[1] - fire_pos[1]
        vx2 = other_pos[0] - fire_pos[0]
        vy2 = other_pos[1] - fire_pos[1]

        mag1 = math.hypot(vx1, vy1)
        mag2 = math.hypot(vx2, vy2)

        if mag1 < 1e-3 or mag2 < 1e-3:
            return True

        dot = vx1 * vx2 + vy1 * vy2
        cos_angle = dot / (mag1 * mag2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angle = math.acos(cos_angle)

        return angle <= self.FIRE_SAME_SIDE_MAX_ANGLE

    def _reset_avoidance_state(self, cooldown: float = 0.0):
        """Nullstill tilstand for møte med annen robot."""
        self.avoidance_active = False
        self.avoidance_mode = None
        self._avoidance_logged = False
        self.robot_memory.other_robot_position = None
        self.other_robot_id = None
        self.other_robot_number = None
        self.other_robot_yaw = None
        self.avoidance_release_time = 0.0
        self.avoidance_cooldown_until = self.get_time_seconds() + cooldown
        try:
            self.wall_follower.start_robot_encounter('clear')
        except Exception:
            pass

    def _big_fire_key(self, position: tuple, precision: float = 0.5) -> tuple:
        """Generer en avrundet nøkkel for en Big Fire-posisjon."""
        return (round(position[0] / precision) * precision,
                round(position[1] / precision) * precision)