"""
ROS2 Bridge Node - Subscribes to navigation topics and serves data via WebSocket.
Handles map (all QoS profiles), lidar scan, TF, and Nav2 status.
"""

import asyncio
import base64
import json
import math
import struct
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from action_msgs.msg import GoalStatusArray


class RosBridge(Node):
    def __init__(self):
        super().__init__('nav_dashboard_bridge')
        self.get_logger().info('Starting Navigation Dashboard Bridge...')

        self.cb_group = ReentrantCallbackGroup()

        # State storage
        self._map_data: Optional[dict] = None
        self._map_update_time: float = 0
        self._scan_data: Optional[dict] = None
        self._robot_pose: Optional[dict] = None
        self._odom_data: Optional[dict] = None
        self._nav_status: str = 'idle'
        self._global_plan: Optional[list] = None
        self._local_plan: Optional[list] = None
        self._battery: Optional[dict] = None
        self._cmd_vel: Optional[dict] = None
        self._connected_clients: set = set()
        self._ws_queues: list = []  # list of asyncio.Queue for each WS client
        self._lock = threading.Lock()

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Subscribers with multiple QoS to catch all publishers ---

        # Map - try both transient_local (typical) and volatile
        map_qos_transient = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        map_qos_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos_transient,
            callback_group=self.cb_group
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos_volatile,
            callback_group=self.cb_group
        )

        # Costmap (global)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb,
            map_qos_volatile, callback_group=self.cb_group
        )

        # Scan - sensor QoS (best effort)
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, scan_qos,
            callback_group=self.cb_group
        )

        # Odom
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, scan_qos,
            callback_group=self.cb_group
        )

        # Nav2 global plan
        plan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Path, '/plan', self._global_plan_cb, plan_qos,
            callback_group=self.cb_group
        )
        self.create_subscription(
            Path, '/local_plan', self._local_plan_cb, scan_qos,
            callback_group=self.cb_group
        )

        # Nav2 status
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._nav_status_cb, scan_qos,
            callback_group=self.cb_group
        )

        # Battery (optional)
        self.create_subscription(
            BatteryState, '/battery_state', self._battery_cb, scan_qos,
            callback_group=self.cb_group
        )

        # Cmd vel for speed display
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, scan_qos,
            callback_group=self.cb_group
        )

        # Publishers
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self._cancel_pub = self.create_publisher(
            GoalStatusArray, '/navigate_to_pose/_action/cancel_goal', 10
        )

        # Timer for TF lookups
        self.create_timer(0.05, self._tf_timer_cb, callback_group=self.cb_group)

        self.get_logger().info('Bridge node ready. Subscriptions active.')

    # ─── Callbacks ───────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        now = time.time()
        if now - self._map_update_time < 0.5:
            return  # debounce duplicate QoS subs

        info = msg.info
        # Compress map: encode as base64 bytes
        raw = bytes(msg.data)
        encoded = base64.b64encode(raw).decode('ascii')

        with self._lock:
            self._map_data = {
                'width': info.width,
                'height': info.height,
                'resolution': info.resolution,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'origin_yaw': _yaw_from_quat(info.origin.orientation),
                'data': encoded,
            }
            self._map_update_time = now
        self._push_event('map', self._map_data)

    def _costmap_cb(self, msg: OccupancyGrid):
        info = msg.info
        raw = bytes(msg.data)
        encoded = base64.b64encode(raw).decode('ascii')
        data = {
            'width': info.width,
            'height': info.height,
            'resolution': info.resolution,
            'origin_x': info.origin.position.x,
            'origin_y': info.origin.position.y,
            'data': encoded,
        }
        self._push_event('costmap', data)

    def _scan_cb(self, msg: LaserScan):
        # Look up laser frame → map transform so scan renders correctly
        # even when the laser frame is rotated relative to base_link.
        laser_frame = msg.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform('map', laser_frame, rclpy.time.Time())
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tf_yaw = _yaw_from_quat(tf.transform.rotation)
        except TransformException:
            # Fall back to robot pose if TF not available yet
            with self._lock:
                p = self._robot_pose
            if p is None:
                return
            tx, ty, tf_yaw = p['x'], p['y'], p['yaw']

        cos_tf = math.cos(tf_yaw)
        sin_tf = math.sin(tf_yaw)

        # Downsample scan for performance: take every Nth point
        n = max(1, len(msg.ranges) // 360)
        ranges = msg.ranges[::n]
        angles = [
            msg.angle_min + i * n * msg.angle_increment
            for i in range(len(ranges))
        ]

        # Convert to x,y directly in map frame
        points = []
        for r, a in zip(ranges, angles):
            if msg.range_min < r < msg.range_max:
                lx = r * math.cos(a)
                ly = r * math.sin(a)
                points.append([
                    tx + lx * cos_tf - ly * sin_tf,
                    ty + lx * sin_tf + ly * cos_tf,
                ])

        with self._lock:
            self._scan_data = {
                'points': points,
                'frame': 'map',
                'range_min': msg.range_min,
                'range_max': msg.range_max,
            }
        self._push_event('scan', self._scan_data)

    def _odom_cb(self, msg: Odometry):
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        with self._lock:
            self._odom_data = {
                'linear_x': lin.x,
                'linear_y': lin.y,
                'angular_z': ang.z,
            }

    def _global_plan_cb(self, msg: Path):
        # Downsample path
        n = max(1, len(msg.poses) // 200)
        pts = [
            [p.pose.position.x, p.pose.position.y]
            for p in msg.poses[::n]
        ]
        with self._lock:
            self._global_plan = pts
        self._push_event('global_plan', {'points': pts})

    def _local_plan_cb(self, msg: Path):
        n = max(1, len(msg.poses) // 100)
        pts = [
            [p.pose.position.x, p.pose.position.y]
            for p in msg.poses[::n]
        ]
        with self._lock:
            self._local_plan = pts
        self._push_event('local_plan', {'points': pts})

    def _nav_status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            status = 'idle'
        else:
            last = msg.status_list[-1].status
            status_map = {
                1: 'accepted',
                2: 'executing',
                4: 'succeeded',
                5: 'canceled',
                6: 'aborted',
            }
            status = status_map.get(last, f'unknown({last})')

        with self._lock:
            self._nav_status = status
        self._push_event('nav_status', {'status': status})

    def _battery_cb(self, msg: BatteryState):
        with self._lock:
            self._battery = {
                'voltage': round(msg.voltage, 2),
                'percentage': round(msg.percentage * 100, 1),
            }

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._cmd_vel = {
                'linear_x': round(msg.linear.x, 3),
                'angular_z': round(msg.angular.z, 3),
            }

    def _tf_timer_cb(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            trans = t.transform.translation
            yaw = _yaw_from_quat(t.transform.rotation)
            with self._lock:
                self._robot_pose = {
                    'x': trans.x,
                    'y': trans.y,
                    'yaw': yaw,
                }
            self._push_event('pose', self._robot_pose)
        except TransformException:
            pass

    # ─── WebSocket push ──────────────────────────────────────

    def _push_event(self, event_type: str, data: dict):
        msg = json.dumps({'type': event_type, **data})
        with self._lock:
            dead = []
            for q in self._ws_queues:
                try:
                    # Non-blocking put, drop if full
                    q.put_nowait(msg)
                except asyncio.QueueFull:
                    pass
                except Exception:
                    dead.append(q)
            for q in dead:
                self._ws_queues.remove(q)

    def register_ws_queue(self) -> asyncio.Queue:
        q = asyncio.Queue(maxsize=200)
        with self._lock:
            self._ws_queues.append(q)
        return q

    def unregister_ws_queue(self, q: asyncio.Queue):
        with self._lock:
            if q in self._ws_queues:
                self._ws_queues.remove(q)

    def get_full_state(self) -> dict:
        with self._lock:
            return {
                'map': self._map_data,
                'scan': self._scan_data,
                'pose': self._robot_pose,
                'odom': self._odom_data,
                'nav_status': self._nav_status,
                'global_plan': self._global_plan,
                'local_plan': self._local_plan,
                'battery': self._battery,
                'cmd_vel': self._cmd_vel,
            }

    # ─── Actions (called from web) ───────────────────────────

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = _quat_from_yaw(yaw)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(f'Published initial pose: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')

    def publish_nav_goal(self, x: float, y: float, yaw: float):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation = _quat_from_yaw(yaw)
        self._goal_pub.publish(msg)
        self.get_logger().info(f'Published nav goal: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')

    def cancel_nav(self):
        msg = GoalStatusArray()
        self._cancel_pub.publish(msg)
        self.get_logger().info('Published nav cancel')


# ─── Helpers ─────────────────────────────────────────────────

def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q
