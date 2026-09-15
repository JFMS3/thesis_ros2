import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from thesis_interfaces.msg import PlatformState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from thesis_mpc_controller.kalman_filter import PositionVelocityKalmanFilter
import os
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory

class PlatformKalmanFilterNode(Node):
    """Subscribes to platform topic, publishes velocity estimates"""
    def __init__(self):
        super().__init__('platform_kalman_filter')
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.declare_parameter('R_pos', [1.0] * 9) 
        self.declare_parameter('sigma_accel', [0.1, 0.1, 0.1])
        self.declare_parameter('P_pos_init', 0.1)
        self.declare_parameter('P_vel_init', 0.1)
        self.declare_parameter('nis_threshold', 11.34)
        self.declare_parameter('max_dt', 0.1)

        R_pos_raw = self.get_parameter('R_pos').value
        R_pos = np.array(R_pos_raw, dtype=float).reshape(3, 3)
        sigma_accel = [float(v) for v in self.get_parameter('sigma_accel').value]
        P_pos_init = float(self.get_parameter('P_pos_init').value)
        P_vel_init = float(self.get_parameter('P_vel_init').value)
        self.max_dt = self.get_parameter('max_dt').value
        self.nis_threshold = self.get_parameter('nis_threshold').value

        self.kf = PositionVelocityKalmanFilter(
            R_pos,
            sigma_accel,
            P_pos_init,
            P_vel_init,
            self.max_dt
        )
        self.nis_threshold = self.get_parameter('nis_threshold').value
        self.prev_stamp = None

        self.create_subscription(PlatformState, '/measured_platform_state', self.on_measurement, state_qos)
        self.pub = self.create_publisher(PlatformState, '/full_platform_state', 10)
        self.nis_pub = self.create_publisher(Float64, '/observed_nis', 10)
        self.get_logger().info("Platform Kalman filter has begun!")


    def on_measurement(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.kf.initialised:
            self.kf.initialise(msg.position)
            self.prev_stamp = stamp
            return
        
        dt = stamp - self.prev_stamp
        self.prev_stamp = stamp
        if not (0.0 < dt <= self.max_dt):
            self.get_logger().warn(f"Bad dt of {dt:.4f}s, skipping...")
            return

        self.kf.predict(dt)
        nis, ok = self.kf.update(msg.position, nis_threshold=self.nis_threshold)
        if not ok:
            self.get_logger().warn(f"NIS {nis:.2f} > threshold, so predicting not updating here...")

        observed_state = PlatformState()
        observed_state.header = msg.header
        x = self.kf.get_state()
        observed_state.position = [x[i] for i in range(3)]
        observed_state.velocity = [x[i] for i in range(3,6)]
        observed_state.attitude = msg.attitude
        self.pub.publish(observed_state)
        self.nis_pub.publish(Float64(data=nis))
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PlatformKalmanFilterNode())
    rclpy.shutdown()
