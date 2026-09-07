import rclpy
from rclpy.node import Node
from thesis_interfaces.msg import PlatformState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
import math

class PlatformController(Node):
    def __init__(self):
        super().__init__('platform_controller')
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', cmd_qos)
        self.subcriber = self.create_subscription(PlatformState, 'measured_platform_state', self.state_callback, state_qos)
        self.timer = self.create_timer(0.05, self.circle_control_loop)

        self.centre = None
        self.radius = 0.8
        self.linear_speed = 0.1
        self.omega = self.linear_speed / self.radius

        # actuator limits
        self.MAX_V = 0.3
        self.MAX_W = 1.9

        self.correction_gain = 0.05
        self.max_correction = 0.03
        self.YAW_JUMP_LIMIT = math.radians(90)

        self.pose = None
        self.start_time = None
        self.stale_count = 0
        self.get_logger().info("Platform Controller node has begun!")
        

    def state_callback(self, msg: PlatformState):
        prev_pose = self.pose
        x = msg.position[0]
        y = msg.position[1]
        yaw = msg.attitude[2]
        if math.isnan(x) or math.isnan(y) or math.isnan(yaw):
            return

        if prev_pose is not None:
            yaw_diff = math.atan2(math.sin(yaw-prev_pose[2]), math.cos(yaw-prev_pose[2]))
            if abs(yaw_diff) > self.YAW_JUMP_LIMIT:
                return
            
        self.pose = (x, y, yaw)
        self.last_pose_time = self.get_clock().now()
        if self.centre is None:
            self.centre = (x - self.radius, y)
            self.start_time = self.get_clock().now()
            self.yaw_offset = yaw - math.pi/2

        if prev_pose is not None:
            if abs(x - prev_pose[0]) < 1e-5 and abs(y - prev_pose[1]) < 1e-5:
                self.stale_count += 1
            else:
                self.stale_count = 0


    def circle_control_loop(self):
        if self.pose is None or self.centre is None:
            return

        now = self.get_clock().now()
        dt_pose = (now - self.last_pose_time).nanoseconds * 1e-9

        if self.stale_count >= 5:
            self.get_logger().info(f"Stale count: {self.stale_count}")
            self.publisher.publish(TwistStamped())
            return
        if dt_pose > 0.2:
            self.publisher.publish(TwistStamped())
            return

        x, y, yaw = self.pose
        t = (now - self.start_time).nanoseconds * 1e-9
        theta_ref = self.omega * t

        cx, cy = self.centre
        ref_x = cx + self.radius * math.cos(theta_ref)
        ref_y = cy + self.radius * math.sin(theta_ref)
        ref_yaw = theta_ref + math.pi/2 + self.yaw_offset

        dx = ref_x - x
        dy = ref_y - y
        dyaw = math.atan2(math.sin(ref_yaw - yaw), math.cos(ref_yaw - yaw))

        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy

        correction = max(-self.max_correction, min(self.max_correction, self.correction_gain * ey))
        v = max(-self.MAX_V, min(self.MAX_V, self.linear_speed))
        w = max(-self.MAX_W, min(self.MAX_W, self.omega + correction))

        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.publisher.publish(cmd)
        self.get_logger().info(f"pose=({x:.3f}, {y:.3f}, {yaw:.3f}) theta_ref={theta_ref:.3f} ref_yaw={ref_yaw:.3f} dyaw={dyaw:.3f} ex={ex:.3f} ey={ey:.3f} v={v:.3f} w={w:.3f}")
    

def main(args=None):
    rclpy.init(args=args)
    node = PlatformController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
