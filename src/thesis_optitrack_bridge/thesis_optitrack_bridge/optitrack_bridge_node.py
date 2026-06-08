import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mocap_optitrack.msg import RigidBodyArray
from thesis_interfaces.msg import QuadcopterState, PlatformState
from .frame_transformer import FrameTransformer
import numpy as np

class OptitrackBridgeNode(Node):
    def __init__(self):
        super().__init__('optitrack_bridge')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.declare_parameter('drone_body_id', 1)
        self.declare_parameter('platform_body_id', 2)
        self.drone_id = self.get_parameter('drone_body_id').value
        self.platform_id = self.get_parameter('platform_body_id').value

        self.rigid_body_subscription = self.create_subscription(
            RigidBodyArray,
            '/mocap_optitrack/rigid_bodies',
            self.rigid_body_callback,
            qos
        )

        self.drone_publisher = self.create_publisher(QuadcopterState, 'quadcopter_state', 10)
        self.platform_publisher = self.create_publisher(PlatformState, 'platform_state', 10)

        self.transformer = FrameTransformer()
        self.get_logger().info("Optitrack bridge node has begun!")


    def rigid_body_callback(self, msg):
        for body in msg.rigid_bodies:
            if body.id == self.drone_id:
                self.publish_drone(body)
            elif body.if == self.platform_id:
                self.publish_platform(body)

    
    def publish_drone(self, body):
        phi, theta, _ = self.transformer.quat_to_euler(body.pose.orientation)
        
        drone_msg = QuadcopterState()
        drone_msg.header.stamp = self.get_clock().now().to_msg()
        drone_msg.header.frame_id = 'WORLD'
        drone_msg.position = [
            body.pose.position.x,
            body.pose.position.y,
            body.pose.position.z
        ]
        drone.msg.velocity = [0, 0, 0]
        drone_msg.attitude = [phi, theta]
        self.drone_publisher.publish(drone_msg)
        

    def publish_platform(self, body):
        platform_msg = PlatformState()
        platform_msg.header.stamp = self.get_clock().now().to_msg()
        platform_msg.header.frame_id = 'WORLD'
        platform_msg.position = [
            body.pose.position.x,
            body.pose.position.y,
            body.pose.position.z
        ]
        platform_msg.msg.velocity = [0, 0, 0]
        self.platform_publisher.publish(platform_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()