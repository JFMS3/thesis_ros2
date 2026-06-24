
import numpy as np
import rclpy
from rclpy.node import Node
from thesis_interfaces.msg import QuadcopterState


class SyntheticSource(Node):
    def __init__(self):
        super().__init__('synthetic_source')
        self.pub = self.create_publisher(QuadcopterState, '/measured_quadcopter_state', 10)
        self.R_std = np.sqrt([1.0e-5, 1.0e-5, 1.0e-5])   # match your R_pos_diag
        self.dt, self.t = 0.01, 0.0                       # 100 Hz, mocap-like
        self.create_timer(self.dt, self.tick)

    def truth(self, t):
        pos = np.array([0.3*np.sin(0.5*t), 0.2*t, 1.0])
        vel = np.array([0.15*np.cos(0.5*t), 0.2, 0.0])    # analytic ground truth
        return pos, vel

    def tick(self):
        self.t += self.dt
        pos, _ = self.truth(self.t)
        noisy = pos + self.R_std * np.random.standard_normal(3)
        if np.random.random() < 0.01:                     # 1% outliers to trip the gate
            noisy[np.random.randint(3)] += 0.05

        msg = QuadcopterState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = noisy.tolist()
        msg.attitude = [0.0, 0.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SyntheticSource())
    rclpy.shutdown()