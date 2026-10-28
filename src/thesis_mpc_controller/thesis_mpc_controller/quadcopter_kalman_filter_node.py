import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from thesis_interfaces.msg import QuadcopterState, AttitudeIMU
from rclpy.qos import QoSProfile, ReliabilityPolicy
from .kalman_filter import PositionVelocityKalmanFilter
import numpy as np
import csv
from pathlib import Path
from datetime import datetime

class QuadcopterKalmanFilterNode(Node):
    """Subscribes to quadcopter_state topic, publishes velocity estimates"""
    def __init__(self):
        super().__init__('quadcopter_kalman_filter')
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
                
        self.declare_parameter('R_pos', [
            4.05334208e-09, 4.08163319e-09, 5.91631335e-09,
            4.08163319e-09, 5.55542538e-09, 5.95189493e-09,
            5.91631335e-09, 5.95189493e-09, 8.73493455e-09
        ]) 
        self.declare_parameter('sigma_accel', [0.6, 0.45, 0.6])
        self.declare_parameter('P_pos_init', 0.001)
        self.declare_parameter('P_vel_init', 0.1)
        self.declare_parameter('nis_threshold', 16)
        self.declare_parameter('max_dt', 0.1)
        self.declare_parameter('R_pos_scalar', 250)

        R_pos_scalar = float(self.get_parameter('R_pos_scalar').value)
        R_pos_raw = self.get_parameter('R_pos').value
        R_pos = np.asarray(R_pos_raw, dtype=float).reshape(3, 3) * R_pos_scalar # measured R_pos seems way too small
        R_pos = R_pos * np.eye(3) # taking diagonal elements for now seems coupled axes not helping
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
        self.prev_quadcopter_state_stamp = None

        self.create_subscription(QuadcopterState, '/measured_quadcopter_state', self.on_measurement, state_qos)
        self.pub = self.create_publisher(QuadcopterState, '/full_quadcopter_state', 10)
        self.nis_pub = self.create_publisher(Float64, '/observed_nis', 10)
        self.get_logger().info("Quadcopter Kalman filter has begun!")

        self.imu_phi = None
        self.imu_theta = None
        self.prev_imu_stamp = None
        self.imu_attitude_subscription = self.create_subscription(
            AttitudeIMU, '/attitude_imu', self.imu_attitude_callback, state_qos
        )

        self.declare_parameter('log_dir', '')
        log_dir = self.get_parameter('log_dir').value
        if log_dir:
            log_dir = Path(log_dir)
        else:
            log_dir = Path(f'log/{datetime.now().strftime("velocity_kf_%m-%d_%H-%M")}')
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f'quadcopter_kf_{timestamp}.csv'

        self.kf_log_file = open(
            log_path, 'w', newline='', buffering=1
        )
        self.get_logger().info(f"Saved csv to {log_path}")
        self.kf_csv_writer = csv.writer(self.kf_log_file)
        self.kf_csv_writer.writerow([
            't', 'dt', 'measured_x', 'measured_y', 'measured_z',
            'estimated_x', 'estimated_y', 'estimated_z',
            'estimated_vx', 'estimated_vy', 'estimated_vz',
            'nis', 'measurement_accepted',
            "innovation_x", "innovation_y", "innovation_z",
            "innovation_covariance_x", "innovation_covariance_y", "innovation_covariance_z",
        ])


    def imu_attitude_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_imu_stamp is not None:
            if stamp <= self.prev_imu_stamp:
                return
        self.prev_imu_stamp = stamp
        self.imu_phi = msg.imu_phi
        self.imu_theta = msg.imu_theta


    def on_measurement(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self.kf.initialised:
            self.kf.initialise(msg.position)
            self.prev_quadcopter_state_stamp = stamp
            return
        
        dt = stamp - self.prev_quadcopter_state_stamp
        if dt <= 0:
            return
        
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_imu_stamp is None or now - self.prev_imu_stamp > 0.15:
            self.get_logger().warn(f"Measured quadcopter state stale for {dt:.4f}s, skipping...", throttle_duration_sec = 1.0)
            return
        
        self.prev_quadcopter_state_stamp = stamp
        remaining_time = dt
        while remaining_time > 1e-9:
            timestep = min(remaining_time, self.max_dt)
            self.kf.predict(timestep)
            remaining_time -= timestep

        nis, ok = self.kf.update(msg.position, nis_threshold=None)# previously self.nis_threshold
        
        observed_state = QuadcopterState()
        observed_state.header = msg.header
        x = self.kf.get_state()
        innovation_x, innovation_y, innovation_z = self.kf.innovation
        innovation_covar_x, innovation_covar_y, innovation_covar_z = np.diag(self.kf.innovation_covariance)


        self.kf_csv_writer.writerow([
            stamp, dt, msg.position[0], msg.position[1], msg.position[2],
            x[0], x[1], x[2], x[3], x[4], x[5], nis, int(ok),
            innovation_x, innovation_y, innovation_z,
            innovation_covar_x, innovation_covar_y, innovation_covar_z
        ])
        observed_state.position = [x[i] for i in range(3)]
        observed_state.velocity = [x[i] for i in range(3,6)]
        observed_state.attitude = [self.imu_phi, self.imu_theta]
        self.pub.publish(observed_state)
        self.nis_pub.publish(Float64(data=nis))


def main():
    rclpy.init()
    node = QuadcopterKalmanFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.kf_log_file.close()
        node.destroy_node()
        rclpy.shutdown()
