import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from thesis_interfaces.msg import PlatformState
from thesis_mpc_controller.thesis_mpc_controller.kalman_filter import PositionVelocityKalmanFilter
import os
import yaml

class PlatformKalmanFilterNode(Node):
    """Subscribes to platform topic, publishes velocity estimates"""
    def __init__(self):
        super().__init__('platform_kalman_filter_node')
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_path = os.path.abspath(os.path.join(current_dir, "config", "matrix_params.yaml"))
        with open(yaml_path, 'r') as f:
            params = yaml.safe_load(f)
            kf_params = params.get("/platform_kalman_filter").get('ros_parameters')

        R_pos_diag = [float(v) for v in kf_params.get('R_pos_diag')]
        sigma_accel = [float(v) for v in kf_params.get('sigma_accel')]
        P_pos_init = float(kf_params.get('P_pos_init'))
        P_vel_init = float(kf_params.get('P_vel_init'))

        self.declare_parameter('nis_threshold', 11.34)
        self.declare_parameter('max_dt', 0.1)

        self.kf = PositionVelocityKalmanFilter(
            R_pos_diag,
            sigma_accel,
            P_pos_init,
            P_vel_init
        )
        self.nis_threshold = self.get_parameter('nis_threshold').value
        self.max_dt = self.get_parameter('max_dt').value
        self.prev_stamp = None

        self.create_subscription(PlatformState, '/measured_platform_state', self.on_measurement, 10)
        self.pub = self.create_publisher(PlatformState, '/full_platform_state', 10)
        self.nis_pub = self.create_publisher(Float64, '/observed_nis', 10)


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

def main():
    rclpy.init()
    rclpy.spin(PlatformKalmanFilterNode())
    rclpy.shutdown()
