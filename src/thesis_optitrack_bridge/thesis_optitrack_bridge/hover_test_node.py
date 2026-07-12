import time
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from thesis_interfaces.msg import QuadcopterState

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


RADIO_URI = "radio://0/80/2M/E7E7E7E7E7"
TARGET_HEIGHT = 0.5
HOVER_DURATION = 5
LAND_DURATION = 3

class QuadcopterSequence(Enum):
    WAITING_FOR_STATE = auto()
    STARTING_ESTIMATOR = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVERING = auto()
    LANDING = auto()
    DONE = auto()


class HoverTestNode(Node):
    def __init__(self):
        super().__init__('hover_test_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.got_first_state = False
        self.sequence = QuadcopterSequence.WAITING_FOR_STATE
        self.state_entered_at = self.get_clock().now()

        self.subscription = self.create_subscription(
            QuadcopterState, '/quadcopter_state', self.sequence_callback, qos
        )

        cflib.crtp.init_drivers()
        self.get_logger().info("Connecting to CrazyFlie...")
        self.sync_cf = SyncCrazyflie(RADIO_URI, cf=Crazyflie(rw_cache='./cache'))
        self.sync_cf.open_link()

        self.cf = self.sync_cf.cf
        self.get_logger().info("Radio link set up")

        self.timer = self.create_timer(0.1, self.step_sequence)
        self.get_logger().info("Starting hover, waiting for state estimate")
        

    def sequence_callback(self, msg: QuadcopterState):
        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]
        self.got_first_state = True
        try:
            self.cf.extpos.send_extpos(x, y, z)
        except Exception as e:
            self.get_logger().error(f"Failed to send extpos: {e}")


    def enter_state(self, new_state: QuadcopterSequence):
        self.sequence = new_state
        self.state_entered_at = self.get_clock().now()


    def time_elapsed(self):
        return (self.get_clock().now() - self.state_entered_at).nanoseconds / 1e9
    

    def shutdown(self):
        self.timer.cancel()
        try:
            self.sync_cf.close_link()
        except Exception:
            pass
    

    def step_sequence(self):
        if self.sequence == QuadcopterSequence.WAITING_FOR_STATE:
            if self.got_first_state:
                self.get_logger().info("Got first state, now resetting estimator")
                self.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.05)
                self.cf.param.set_value('kalman.resetEstimation', '0')

                self.enter_state(QuadcopterSequence.STARTING_ESTIMATOR)
            elif self.time_elapsed() > 15:
                self.get_logger().error("No quadcopter state received in time, aborting...")
                self.shutdown()
        elif self.sequence == QuadcopterSequence.STARTING_ESTIMATOR:
            if self.time_elapsed() > 2:
                self.get_logger().info("Arming...")
                self.cf.platform.send_arming_request(True)
                self.enter_state(QuadcopterSequence.ARMING)
        elif self.sequence == QuadcopterSequence.ARMING:
            if self.time_elapsed() > 1:
                self.get_logger().info("Taking off...")
                self.cf.high_level_commander.takeoff(TARGET_HEIGHT, HOVER_DURATION)
                self.enter_state(QuadcopterSequence.TAKEOFF)
        elif self.sequence == QuadcopterSequence.TAKEOFF:
            if self.time_elapsed() > HOVER_DURATION + 0.5:
                self.get_logger().info(f"Beginning hover at {TARGET_HEIGHT}m for {HOVER_DURATION}s")
                self.cf.high_level_commander.go_to(x=0, y=0, z=TARGET_HEIGHT, yaw=0, duration_s=HOVER_DURATION)
                self.enter_state(QuadcopterSequence.HOVERING)
        elif self.sequence == QuadcopterSequence.HOVERING:
            if self.time_elapsed() > HOVER_DURATION:
                self.get_logger().info(f"Landing...")
                self.cf.high_level_commander.land(absolute_height_m=0, duration_s=LAND_DURATION)
                self.enter_state(QuadcopterSequence.LANDING)
        elif self.sequence == QuadcopterSequence.LANDING:
            if self.time_elapsed() > LAND_DURATION + 0.5:
                self.cf.high_level_commander.stop()
                self.get_logger().info(f"Landed")
                self.enter_state(QuadcopterSequence.DONE)
                self.timer.cancel()
        elif self.sequence == QuadcopterSequence.DONE:
            pass


def main():
    rclpy.init()
    node = HoverTestNode()

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        try:
            node.sync_cf.close_link()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
usbipd list
usbipd bind --busid <BUS_ID> (in powershell admin)
usbipd attach --wsl --busid <BUS_ID>
lsusb

colcon build --packages-select thesis_optitrack_bridge
ros2 run thesis_optitrack_bridge hover_test_node

(when done)
usbipd detach --busid <BUS_ID>
'''