import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from thesis_interfaces.msg import QuadcopterState
from .kalman_filter import PositionVelocityKalmanFilter

class KalmanFilterNode(Node):
    """Subscribes to quadcopter_state topic, publishes velocity estimates"""
    def __init__(self):
        super().__init__('kalman_filter_node')
        self.declare_parameter('R_pos_diag', [1.0e-5, 1.0e-5, 1.0e-5])
        self.declare_parameter('sigma_accel', [0.5, 0.5, 0.3])
        self.declare_parameter('P_pos_init', 1.0e-3)
        self.declare_parameter('P_vel_init', 1.0)
        self.declare_parameter('nis_threshold', 11.34)
        self.declare_parameter('max_dt', 0.1)

        self.kf = PositionVelocityKalmanFilter(
            self.get_parameter('R_pos_diag').value,
            self.get_parameter('sigma_accel').value,
            self.get_parameter('P_pos_init').value,
            self.get_parameter('P_vel_init').value
        )
        self.nis_threshold = self.get_parameter('nis_threshold').value
        self.max_dt = self.get_parameter('max_dt').value
        self.prev_stamp = None

        self.create_subscription(QuadcopterState, '/measured_quadcopter_state', self.on_measurement, 10)
        self.pub = self.create_publisher(QuadcopterState, '/full_quadcopter_state', 10)
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

        observed_state = QuadcopterState()
        observed_state.header = msg.header
        x = self.kf.get_state()
        observed_state.position = [x[i] for i in range(3)]
        observed_state.velocity = [x[i] for i in range(3,6)]
        observed_state.attitude = msg.attitude
        self.pub.publish(observed_state)
        self.nis_pub.publish(Float64(data=nis))

def main():
    rclpy.init()
    rclpy.spin(KalmanFilterNode())
    rclpy.shutdown()
