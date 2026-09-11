import rclpy
from rclpy.node import Node
from thesis_interfaces.msg import PlatformState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
import math
import numpy as np

class PlatformController(Node):
    def __init__(self):
        super().__init__('platform_controller')
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', cmd_qos)
        self.subcriber = self.create_subscription(PlatformState, 'measured_platform_state', self.state_callback, state_qos)
        self.timer = self.create_timer(0.05, self.simple_circle_path)

        self.centre = None
        self.starting_xs = []
        self.starting_ys = []
        self.starting_yaws = []
        self.off_r = 0.0
        self.off_t = 0.0
        self.alpha = 0.02

        # actuator limits
        self.MAX_V = 0.3
        self.MAX_W = 1.9
        
        self.declare_parameter('RADIUS', 0.8)
        self.radius = float(self.get_parameter('RADIUS').value)
        self.declare_parameter('LINSPEED', 0.1)
        self.linear_speed = min(self.MAX_V, float(self.get_parameter('LINSPEED').value))
        self.omega = min(self.MAX_W, self.linear_speed / self.radius)

        self.model_radius = self.radius # what the model thinks the radius is

        self.refit_points = [] # once bot has driven a decent arc, fit circle to measured positions
        self.refit_done = False
        self.REFIT_AFTER = 5.0

        self.YAW_JUMP_LIMIT = math.radians(90)
        self.last_pose_time = None
        self.stale_limit = 0.5

        self.pose = None
        self.start_time = None
        self.get_logger().info("Platform Controller node has begun!")
        self.get_logger().info(f"Commanding linspeed of {self.linear_speed} and omega {self.omega}")


    def nominal_at(self, t):
        cx, cy = self.centre
        phi = self.omega * t + self.yaw_offset
        return cx + self.model_radius * math.cos(phi), cy + self.model_radius * math.sin(phi)

    def predict_at(self, t):
        cx, cy = self.centre
        phi = self.omega * t + self.yaw_offset
        r = self.model_radius + self.off_r
        delta_phi = self.off_t / self.model_radius
        return cx + r * math.cos(phi + delta_phi), cy + r * math.sin(phi + delta_phi)


    def initial_calibration(self, x, y, yaw):
        self.starting_xs.append(x)
        self.starting_ys.append(y)
        self.starting_yaws.append(yaw)

        if len(self.starting_xs) <= 5:
            return
        x_avg = sum(self.starting_xs) / len(self.starting_xs)
        y_avg = sum(self.starting_ys) / len(self.starting_ys)
        yaw_avg = sum(self.starting_yaws) / len(self.starting_yaws)
        self.centre = (x_avg - self.radius * math.sin(yaw_avg), y_avg + self.radius * math.cos(yaw_avg))
        self.start_time = self.get_clock().now()
        self.yaw_offset = yaw_avg - math.pi/2
        self.get_logger().info(f"Centre: {self.centre}, Yaw Offset: {self.yaw_offset}")
        

    def refit_circle(self):

        arr = np.array(self.refit_points, dtype=float)
        ts, xs, ys = arr[:, 0], arr[:, 1], arr[:, 2]

        # fits cx, cy and radius to measured x, y via linear least squares (just expanding (x-cx)^2+(y-cy)^2=r^2)
        A = np.c_[2 * xs, 2 * ys, np.ones(len(xs))]
        (cx, cy, c), *_ = np.linalg.lstsq(A, xs ** 2 + ys ** 2, rcond=None)
        r = math.sqrt(max(c + cx * cx + cy * cy, 1e-9))

        if len(ts) < 20 or not (0.5 * self.radius < r < 2.0 * self.radius):
            self.get_logger().warn(f"Refit rejected: {len(ts)} samples, fitted radius {r:.3f} m ")
            self.REFIT_AFTER += 5
            return

        self.refit_done = True
        moved = math.hypot(cx - self.centre[0], cy - self.centre[1])
        self.centre = (float(cx), float(cy))
        self.model_radius = r
        self.yaw_offset = (math.atan2(ys[-1] - cy, xs[-1] - cx) - self.omega * float(ts[-1]))
        self.off_r = 0.0
        self.off_t = 0.0

        self.get_logger().info(
            f"Refit: centre ({cx:+.3f},{cy:+.3f}) moved {moved:.3f} m, radius {r:.3f} (cmd {self.radius:.3f})")
        if moved > 0.5 * self.radius:
            self.get_logger().warn(f"Yaw-based centre was off by {moved:.3f} m - probably mocap error")


    def state_callback(self, msg: PlatformState):
        prev_pose = self.pose
        x = float(msg.position[0])
        y = float(msg.position[1])
        yaw = -float(msg.attitude[2])
        if math.isnan(x) or math.isnan(y) or math.isnan(yaw):
            return

        if prev_pose is not None:
            yaw_diff = math.atan2(math.sin(yaw-prev_pose[2]), math.cos(yaw-prev_pose[2]))
            if abs(yaw_diff) > self.YAW_JUMP_LIMIT:
                return
            
        self.pose = (x, y, yaw)
        self.last_pose_time = self.get_clock().now()

        if self.centre is None:
            self.initial_calibration(x, y, yaw)
            return


        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if not self.refit_done:
            self.refit_points.append((t, x, y))
            if t > self.REFIT_AFTER:
                self.refit_circle()

        nx, ny = self.nominal_at(t)
        phi = self.omega * t + self.yaw_offset

        rx, ry = x - nx, y - ny
        radial = rx * math.cos(phi) + ry * math.sin(phi)
        tangential = -rx * math.sin(phi) + ry * math.cos(phi)

        self.off_r += self.alpha * (radial - self.off_r)
        self.off_t += self.alpha * (tangential - self.off_t)

      
    def pose_age(self):
        # seconds since last accepted mocap measurement
        if self.last_pose_time is None: return float('inf')
        return (self.get_clock().now() - self.last_pose_time).nanoseconds * 1e-9


    def simple_circle_path(self):
        if self.centre is None: return
        
        now = self.get_clock().now()
        delta_t = (now - self.start_time).nanoseconds * 1e-9

        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(self.linear_speed)
        cmd.twist.angular.z = float(self.omega)
        self.publisher.publish(cmd)

        lookahead = 2.0
        pred_x, pred_y = self.predict_at(delta_t + lookahead)
        nx, ny = self.nominal_at(delta_t)
        age = self.pose_age()

        self.get_logger().info(f"Current: ({self.pose[0]:.3f}, {self.pose[1]:.3f}) | Expected: ({nx:.3f}, {ny:.3f}) | Offset: r={self.off_r:.3f}, t={self.off_t:.3f} | Predicted: ({pred_x:.3f}, {pred_y:.3f}) | age={age:.2f}s")


def main(args=None):
    rclpy.init(args=args)
    node = PlatformController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
