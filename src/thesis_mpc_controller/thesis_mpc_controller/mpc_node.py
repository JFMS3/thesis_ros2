import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from thesis_interfaces.msg import QuadcopterState, PlatformState, ControllerMode, MPCCommand
from .kalman_filter import PositionVelocityKalmanFilter
from .quadcopter_solver import setup_ocp_solver
import numpy as np
from math import isfinite
import csv
from datetime import datetime
from pathlib import Path


class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        mpc_active_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        self.declare_parameter('fs', 20)
        self.fs = self.get_parameter('fs').value
        self.declare_parameter('N_horizon', 40)
        self.N_horizon = self.get_parameter('N_horizon').value

        self.h = 1/self.fs
        self.Tf = self.N_horizon*self.h
        Ax, Ay, Az = 0.2, 0.2, 0.4
        tau_phi, tau_theta = 0.2, 0.2
        self.declare_parameter('Ax', 0.6)
        self.declare_parameter('Ay', 0.6)
        self.declare_parameter('Az', 0.6)
        self.declare_parameter('tau_phi', 94.6e-4)
        self.declare_parameter('tau_theta', 94.6e-4)

        Ax = float(self.get_parameter('Ax').value)
        Ay = float(self.get_parameter('Ay').value)
        Az = float(self.get_parameter('Az').value)
        tau_phi = float(self.get_parameter('tau_phi').value)
        tau_theta = float(self.get_parameter('tau_theta').value)

        self.vel_lpf_alpha = 0.3 # higher = more smoothing, more lag
        self.vel_filt = np.zeros(3)
        
        m = 40e-3
        x0_init = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.ocp, self.ocp_solver = setup_ocp_solver(
            x0_init, self.N_horizon, self.Tf, Ax, Ay, Az, tau_phi, tau_theta, m
        )

        self.nx = self.ocp.dims.nx
        self.nu = self.ocp.dims.nu
        self.x_hat = None
        self.platform_pos = None
        self.platform_vel = None
        self.mode = ControllerMode.TRACKING_MODE
        self.timer = self.create_timer(self.h, self.control_loop)

        
        self.declare_parameter('TRACK_HEIGHT', 1.0)
        self.TRACK_HEIGHT = float(self.get_parameter('TRACK_HEIGHT').value) # just make drone hover 1m above platform for now
        self.prev_drone_pos = None # for now not using drone kalman filter
        self.prev_drone_stamp = None
        

        self.mpc_enabled = False

        self.quadcopter_subscription = self.create_subscription(
            QuadcopterState,
            '/measured_quadcopter_state',
            self.quadcopter_callback,
            qos
        )
    
        self.platform_subscription = self.create_subscription(
            PlatformState,
            '/full_platform_state',
            self.platform_callback,
            qos
        )

        self.cmd_publisher = self.create_publisher(
            MPCCommand, '/mpc_cmd', 10
        )

        self.controller_mode_subscription = self.create_subscription(
            ControllerMode,
            '/controller_mode',
            self.controller_mode_callback,
            mpc_active_qos
        )

        log_dir = Path.home() / "preliminary_mpc_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f"mpc_{timestamp}.csv"

        self.log_file = open(log_path, "w", newline="", buffering=1)
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "time", "x", "y", "z", "vx", "vy", "vz",
            "phi", "theta", "ref_x", "ref_y", "ref_z",
            "phi_cmd", "theta_cmd", "thrust_dev", "solver_status",
        ])
        self.get_logger().info(f"Logging MPC data to {log_path}")


    def quadcopter_callback(self, msg: QuadcopterState):
        pos = np.array(msg.position, dtype=float)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_drone_pos is None:
            vel_raw = np.zeros(3)
        else:
            dt = stamp - self.prev_drone_stamp

            if 0.0 < dt <= 0.1:
                vel_raw = (pos - self.prev_drone_pos) / dt
            else:
                vel_raw = np.zeros(3)

        self.vel_filt = self.vel_lpf_alpha * vel_raw + (1-self.vel_lpf_alpha) * self.vel_filt
        vel = self.vel_filt

        self.x_hat = np.array([
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            msg.attitude[0], msg.attitude[1]
        ])
        self.prev_drone_pos = pos
        self.prev_drone_stamp = stamp


    def platform_callback(self, msg):
        self.platform_pos = np.array(msg.position, dtype=float)
        self.platform_vel = np.array(msg.velocity, dtype=float)


    def controller_mode_callback(self, msg):
        was_enabled = self.mpc_enabled
        self.mpc_enabled = msg.mpc_active == ControllerMode.MPC_ACTIVE
        if self.mpc_enabled and not was_enabled:
            self.get_logger().info("MPC mode active")
            self.ref_offset = None
            self.mpc_enabled_at = self.get_clock().now()


    def control_loop(self):
        if not self.mpc_enabled:
            return
        
        if self.platform_pos is None or self.platform_vel is None or self.x_hat is None: 
            self.get_logger().warn(
                "Waiting for state estimate and platform state", throttle_duration_sec=2
            )
            return

        quadcopter_state_age = self.get_clock().now().nanoseconds * 1e-9 - self.prev_drone_stamp
        if quadcopter_state_age > 0.15:
            self.get_logger().warn(f"Haven't received valid quadcopter state for {quadcopter_state_age:.2f}s, not solving MPC", throttle_duration_sec=1.0)
            return

        refs = self.get_platform_preview()

        for k in range(self.N_horizon):
            yref_k = np.concatenate([refs[k, :], np.zeros(self.nu)])
            self.ocp_solver.cost_set(k, 'yref', yref_k)
        self.ocp_solver.cost_set(self.N_horizon, 'yref', refs[self.N_horizon, :])

        u_opt = self.ocp_solver.solve_for_x0(x0_bar=self.x_hat, fail_on_nonzero_status=False)
        solver_status = self.ocp_solver.status
        now = self.get_clock().now()

        self.log_writer.writerow([
            now.nanoseconds * 1e-9, self.x_hat[0], self.x_hat[1], self.x_hat[2],
            self.x_hat[3], self.x_hat[4], self.x_hat[5], self.x_hat[6], self.x_hat[7],
            refs[0, 0], refs[0, 1], refs[0, 2], u_opt[0], u_opt[1], u_opt[2], solver_status,
        ])

        if self.ocp_solver.status != 0:
            self.get_logger().warn(f'MPC solver failed with status {self.ocp_solver.status}')
            return

        cmd = MPCCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.phi_cmd = float(u_opt[0])
        cmd.theta_cmd = float(u_opt[1])
        cmd.thrust_dev = float(u_opt[2])
        if not isfinite(cmd.phi_cmd) or not isfinite(cmd.theta_cmd) or not isfinite(cmd.thrust_dev):
            self.get_logger().error("Invalid MPC command")
            return
        self.cmd_publisher.publish(cmd)


        
    def get_platform_preview(self):
        refs = np.zeros((self.N_horizon + 1, self.nx))
        for i in range(self.N_horizon + 1):
            tau = i * self.h
            p_pred = self.platform_pos # assumes stationary platform
            #p_pred = self.platform_pos + self.platform_vel * tau

            refs[i, 0] = p_pred[0]
            refs[i, 1] = p_pred[1]
            refs[i, 2] = p_pred[2] + self.TRACK_HEIGHT

        return refs


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