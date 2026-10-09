import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from thesis_interfaces.msg import PlatformState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from thesis_mpc_controller.kalman_filter import PositionVelocityKalmanFilter
import csv
from pathlib import Path
from datetime import datetime

import numpy as np
from ament_index_python.packages import get_package_share_directory
from collections import deque
from std_msgs.msg import Bool
from math import sqrt

class PlatformKalmanFilterNode(Node):
    """Subscribes to platform topic, publishes velocity estimates"""
    def __init__(self):
        super().__init__('platform_kalman_filter')
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.declare_parameter('R_pos', [1.0] * 9) 
        self.declare_parameter('sigma_accel', [0.1, 0.1, 0.1])
        self.declare_parameter('P_pos_init', 0.1)
        self.declare_parameter('P_vel_init', 0.1)
        self.declare_parameter('nis_threshold', 16)
        self.declare_parameter('max_dt', 0.1)
        self.declare_parameter('R_pos_scalar', 10)

        R_pos_scalar = float(self.get_parameter('R_pos_scalar').value)
        R_pos_raw = self.get_parameter('R_pos').value
        R_pos = np.asarray(R_pos_raw, dtype=float).reshape(3, 3) * R_pos_scalar
        R_pos = R_pos * np.eye(3)

        sigma_accel = [float(v) for v in self.get_parameter('sigma_accel').value]
        self.P_pos_init = float(self.get_parameter('P_pos_init').value)
        self.P_vel_init = float(self.get_parameter('P_vel_init').value)
        self.max_dt = self.get_parameter('max_dt').value
        self.nis_threshold = self.get_parameter('nis_threshold').value

        self.kf = PositionVelocityKalmanFilter(
            R_pos,
            sigma_accel,
            self.P_pos_init,
            self.P_vel_init,
            self.max_dt
        )
        self.nis_threshold = self.get_parameter('nis_threshold').value
        self.prev_stamp = None

        self.create_subscription(PlatformState, '/measured_platform_state', self.on_measurement, state_qos)
        self.pub = self.create_publisher(PlatformState, '/full_platform_state', 10)
        self.healthline_pub = self.create_publisher(Bool, '/platform_kf_healthline', 10)
        self.healthline_window = deque(maxlen=120)

        self.reject_count = 0
        self.max_rejections = 60
        self.EXPECTED_PLATFORM_HEIGHT = 0.023
        self.last_good_pos = None
        self.last_good_stamp = None

        self.get_logger().info("Platform Kalman filter has begun!")
        self.declare_parameter('log_dir', '')
        log_dir = self.get_parameter('log_dir').value
        if log_dir:
            log_dir = Path(log_dir)
        else:
            log_dir = Path(f'log/{datetime.now().strftime("velocity_kf_%m-%d_%H-%M")}')
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f'platform_kf_{timestamp}.csv'

        self.kf_log_file = open(log_path, 'w', newline='', buffering=1)
        self.get_logger().info(f"Saved platform kalman filter csv to {log_path}")
        self.kf_csv_writer = csv.writer(self.kf_log_file)
        self.kf_csv_writer.writerow([
            't', 'dt', 'measured_x', 'measured_y', 'measured_z',
            'estimated_x', 'estimated_y', 'estimated_z',
            'estimated_vx', 'estimated_vy', 'estimated_vz',
            'nis', 'measurement_accepted'
        ])


    def on_measurement(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = np.array(msg.position, dtype=float)
        if not self.kf.initialised:
            self.kf.initialise(msg.position)
            self.prev_stamp = stamp
            self.last_good_pos = pos.copy()
            self.last_good_stamp = stamp
            return

        dt = 1.0/120.0
        arrival_dt = stamp - self.prev_stamp
        self.prev_stamp = stamp
        if (arrival_dt <= 0):
            self.get_logger().warn(f"Bad arrival dt of {dt:.4f}s, skipping...")
            return

        self.kf.predict(dt)
        elapsed = stamp - self.last_good_stamp
        xy_jump = sqrt((pos[0]-self.last_good_pos[0])**2 + (pos[1]-self.last_good_pos[1])**2)
        z_error = abs(pos[2] - self.EXPECTED_PLATFORM_HEIGHT)
        max_xy_jump = 0.3 * elapsed + 0.03
        plausible_measurement = z_error < 0.05 and elapsed > 0 and xy_jump <= max_xy_jump

        if plausible_measurement:
            nis, ok = self.kf.update(msg.position, nis_threshold=self.nis_threshold)
        else:
            nis, ok = None, False

        if ok:
            self.last_good_pos = pos.copy()
            self.last_good_stamp = stamp

        self.reject_count = 0 if ok else self.reject_count + 1
        self.healthline_window.append(ok)
        healthy = len(self.healthline_window) >= 120 and np.mean(self.healthline_window) >= 0.8
        self.healthline_pub.publish(Bool(data=bool(healthy)))
        
        if self.reject_count >= self.max_rejections:
            self.get_logger().warn("Platform KF has rejected 0.5s of measurements", throttle_duration_sec = 1.0)
            # reset kalman filter if last measurement is sensible
            if abs(msg.position[2] - self.EXPECTED_PLATFORM_HEIGHT) < 0.05:
                self.get_logger().info("Resetting platform kalman filter")
                self.kf.initialise(msg.position)
                self.kf.kf.P = np.diag([self.P_pos_init] * 3 + [self.P_vel_init] * 3)
                self.reject_count = 0
                self.healthline_window.clear()

        observed_state = PlatformState()
        observed_state.header = msg.header
        x = self.kf.get_state()
        self.kf_csv_writer.writerow([
            stamp, dt, msg.position[0], msg.position[1], msg.position[2],
            x[0], x[1], x[2], x[3], x[4], x[5], nis, int(ok)
        ])
        observed_state.position = [x[i] for i in range(3)]
        observed_state.velocity = [x[i] for i in range(3,6)]
        observed_state.attitude = msg.attitude
        self.pub.publish(observed_state)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PlatformKalmanFilterNode())
    rclpy.shutdown()
