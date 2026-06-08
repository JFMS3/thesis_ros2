import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from thesis_interfaces.msg import QuadcopterState, PlatformState, ControllerMode, MPCCommand
from .mpc_solver import MPCSolver
from .observer import Observer
import numpy as np

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.quadcopter_subscription = self.create_subscription(
            QuadcopterState,
            'quadcopter_state',
            self.quadcopter_callback,
            qos
        )
    
        self.platform_subscription = self.create_subscription(
            PlatformState,
            'platform_state',
            self.platform_callback,
            qos
        )

        self.cmd_publisher = self.create_publisher(
            MPCCommand, 'mpc_cmd', 10
        )

        self.declare_parameter('Np', 40)
        self.declare_parameter('Nc', 10)
        self.declare_parameter('fs', 20.0)

        self.fs = self.get_parameter('fs').value
        self.Np = self.get_parameter('Np').value
        self.Nc = self.get_parameter('Nc').value

        self.latest_platform_state = None
        self.observer = Observer()
        self.solver = MPCSolver(Np, Nc)
        self.mode = ControllerMode.TRACKING_MODE
        self.timer = self.create_timer(1.0/self.fs, self.control_loop)


    def quadcopter_callback(self, msg):
        y_meas = np.array([*msg.position, *msg.attitude])
        self.observer.update(y_meas)

    def platform_callback(self, msg):
        self.latest_platform_state = msg

    def control_loop(self):
        if self.latest_platform_state is None: return

        x_hat = self.observer.predict()
        self.mode = self.solver.update_mode(x_hat, self.latest_platform_state)

        u_opt = self.solver.solve(x_hat, self.latest_platform_state, self.mode)
        if u_opt is None:
            self.get_logger().warn('MPC solver failed')
            return

        cmd = MPCCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.phi_cmd = u_opt[0]
        cmd.theta_cmd = u_opt[1]
        cmd.thrust_dev = u_opt[2]
        self.cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()