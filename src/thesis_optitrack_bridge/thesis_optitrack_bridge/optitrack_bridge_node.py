import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Quaternion
from thesis_interfaces.msg import QuadcopterState, PlatformState
from .frame_transformer import FrameTransformer
from NatNetClient import NatNetClient

class OptitrackBridgeNode(Node):
    def __init__(self):
        super().__init__('optitrack_bridge')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.declare_parameter('drone_body_id', 1)
        self.declare_parameter('platform_body_id', 2)
        self.drone_id = self.get_parameter('drone_body_id').value
        self.platform_id = self.get_parameter('platform_body_id').value

        self.declare_parameter('server_address', '192.168.4.105')
        self.declare_parameter('client_address', '192.168.4.105')
        self.declare_parameter('use_multicast', False)
        self.server_address = self.get_parameter('server_address').value
        self.client_address = self.get_parameter('client_address').value
        self.use_multicast = self.get_parameter('use_multicast').value

        self.drone_publisher = self.create_publisher(QuadcopterState, 'quadcopter_state', qos)
        self.platform_publisher = self.create_publisher(PlatformState, 'platform_state', qos)

        self.level_plane = None # ground plane not perfectly flat in Motive
        self.transformer = FrameTransformer()
        self.get_logger().info("Optitrack bridge node has begun!")

        self.client = NatNetClient()
        self.client.set_server_address(self.server_address)
        self.client.set_client_address(self.client_address)
        self.client.set_use_multicast(self.use_multicast)
        self.client.rigid_body_listener = self.rigid_body_callback

        mode = 'multicast' if self.use_multicast else 'unicast'
        self.get_logger().info(f"Connecting to Motive at {self.server_address} ({mode})")
        if not self.client.run("d"):
            self.get_logger().error("NatNet Client failed to start")
            return
        self.get_logger().info("Optitrack bridge has started!")


    def rigid_body_callback(self, new_id, position, rotation):
        self.get_logger().info(f"Got rigid body id = {new_id} with position {position} and rotation {rotation}")
        if new_id == self.drone_id:
            self.publish_drone(position, rotation)
        elif new_id == self.platform_id:
            self.publish_platform(position, rotation)

    
    def publish_drone(self, position, rotation):
        q_now = self.transformer.normalise_quat(rotation)
        if self.level_plane is None:
            self.level_plane = q_now.copy()

        q_relative = self.transformer.quat_multiply(
            q_now,
            self.transformer.quat_conjugate(self.level_plane)
        )

        q = Quaternion()
        q.x, q.y, q.z, q.w = q_relative
        phi, theta, _ = self.transformer.quat_to_euler(q)

        drone_msg = QuadcopterState()
        drone_msg.header.stamp = self.get_clock().now().to_msg()
        drone_msg.header.frame_id = 'WORLD'
        drone_msg.position = [
            float(position[0]),
            float(position[1]),
            float(position[2])
        ]
        drone_msg.velocity = [0.0, 0.0, 0.0]
        drone_msg.attitude = [phi, theta]
        self.drone_publisher.publish(drone_msg)



    def publish_platform(self, position, rotation):
        platform_msg = PlatformState()
        platform_msg.header.stamp = self.get_clock().now().to_msg()
        platform_msg.header.frame_id = 'WORLD'
        platform_msg.position = [
            float(position[0]),
            float(position[1]),
            float(position[2])
        ]
        platform_msg.velocity = [0.0, 0.0, 0.0]
        self.platform_publisher.publish(platform_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()