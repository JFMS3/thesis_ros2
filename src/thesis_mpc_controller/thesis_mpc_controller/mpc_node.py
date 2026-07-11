import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from thesis_interfaces.msg import QuadcopterState, PlatformState, ControllerMode, MPCCommand
from .kalman_filter import Observer
from .quadcopter_solver import setup_ocp_solver
import numpy as np


def generate_platform_preview(px, py, N_horizon, nx, z_hover=2.0):
    ref = np.zeros(nx)
    ref[0] = px
    ref[1] = py
    ref[2] = z_hover
    return np.tile(ref, (N_horizon + 1, 1))


class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.declare_parameter('fs', 20)
        self.fs = self.get_parameter('fs').value
        self.declare_parameter('N_horizon', 40)
        self.N_horizon = self.get_parameter('N_horizon').value

        self.h = 1/self.fs
        self.Tf = self.N_horizon*self.h
        Ax, Ay, Az = 0.2, 0.2, 0.4
        tau_phi, tau_theta = 0.2, 0.2
        m = 40e-3
        x0_init = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.ocp, self.ocp_solver = setup_ocp_solver(
            x0_init, self.N_horizon, self.Tf, Ax, Ay, Az, tau_phi, tau_theta, m
        )

        self.nx = self.ocp.dims.nx
        self.nu = self.ocp.dims.nu
        self.x_hat = None
        self.platform_state = None
        self.mode = ControllerMode.TRACKING_MODE
        self.timer = self.create_timer(self.h, self.control_loop)

        self.quadcopter_subscription = self.create_subscription(
            QuadcopterState,
            '/quadcopter_state',
            self.quadcopter_callback,
            qos
        )
    
        self.platform_subscription = self.create_subscription(
            PlatformState,
            '/platform_state',
            self.platform_callback,
            qos
        )

        self.cmd_publisher = self.create_publisher(
            MPCCommand, '/mpc_cmd', 10
        )


    def quadcopter_callback(self, msg: QuadcopterState):
        self.x_hat = np.array([
            msg.position[0], msg.position[1], msg.position[2],
            msg.velocity[0], msg.velocity[1], msg.velocity[2],
            msg.attitude[0], msg.attitude[1]
        ])

    def platform_callback(self, msg):
        self.platform_state = msg

    def control_loop(self):
        if self.platform_state is None or self.x_hat is None: 
            self.get_logger.warn(
                "Waiting for state estimate and platform state", 
                throttle_duration_sec=2
            )
            return

        px = self.platform_state.position[0]
        py = self.platform_state.position[1]
        refs = generate_platform_preview(px, py, self.N_horizon, self.nx)

        for k in range(self.N_horizon):
            yref_k = np.concatenate([refs[k, :], np.zeros(self.nu)])
            self.ocp_solver.cost_set(k, 'yref', yref_k)
        self.ocp_solver.cost_set(self.N_horizon, 'yref', refs[self.N_horizon, :])

        u_opt = self.ocp_solver.solve_for_x0(x0_bar=self.x_hat, fail_on_nonzero_status=False)
        if self.ocp_solver.status != 0:
            self.get_logger().warn(f'MPC solver failed with status {self.ocp_solver.status}')
            return

        cmd = MPCCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.phi_cmd = float(u_opt[0])
        cmd.theta_cmd = float(u_opt[1])
        cmd.thrust_dev = float(u_opt[2])
        self.cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()