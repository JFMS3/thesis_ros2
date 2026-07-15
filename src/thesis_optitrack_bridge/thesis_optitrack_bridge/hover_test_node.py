import time
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from math import sqrt

from thesis_interfaces.msg import QuadcopterState

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


RADIO_URI = "radio://0/80/2M/E7E7E7E7E7"
TARGET_HEIGHT = 0.25
HOVER_DURATION = 8
LAND_DURATION = 3
MAX_POSSIBLE_SPEED = 3

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
        self.starting_position = []
        self.sequence = QuadcopterSequence.WAITING_FOR_STATE
        self.state_entered_at = self.get_clock().now()
        self.target_z = None
        self.last_valid_pos = None
        self.last_valid_time = None
        self.rejected_count = 0

        self.last_msg_time = None
        self.max_gap_seen = 0
        self.gap_count = 0

        self.subscription = self.create_subscription(
            QuadcopterState, '/measured_quadcopter_state', self.sequence_callback, qos
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
        now = self.get_clock().now()
        if self.last_msg_time is not None:
            gap = (now - self.last_msg_time).nanoseconds/1e9
            if gap > 0.15:
                self.gap_count += 1
                self.get_logger().warn(f"Optitrack gap of {gap*1000:.1f}ms since last message (gap count: {self.gap_count})")
            if gap > self.max_gap_seen:
                self.max_gap_seen = gap
        
        self.last_msg_time = now
        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]

        if not self.got_first_state:
            self.get_logger().info(f"Got starting position {x}, {y}, {z}")
            self.starting_position = [x, y, z]
            self.target_z = z + TARGET_HEIGHT
        elif self.last_valid_pos is not None:
            dt = (now - self.last_valid_time).nanoseconds / 1e9
            if dt > 0:
                dist = sqrt(
                    (x - self.last_valid_pos[0]) ** 2 +
                    (y - self.last_valid_pos[1]) ** 2 +
                    (z - self.last_valid_pos[2]) ** 2
                )
                implied_speed = dist / dt
                if implied_speed > MAX_POSSIBLE_SPEED:
                    self.rejected_count += 1
                    self.get_logger().warn(f"Rejecting optitrack jump of {dist:.3f}m over {dt:.3f}s. Rejected counter: {self.rejected_count}")
                    return

        if z > 2:
            self.cf.commander.send_stop_setpoint()
            self.get_logger().warn("Climbed too high, cutting power!")

        self.got_first_state = True
        self.last_valid_pos = (x, y, z)
        self.last_valid_time = now
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
        self.get_logger().info(f"Max gap: {self.max_gap_seen}, no. gaps: {self.gap_count}")
        self.timer.cancel()
        try:
            self.sync_cf.close_link()
        except Exception:
            pass

    def emergency_land(self):
        if self.sequence in (QuadcopterSequence.DONE, QuadcopterSequence.WAITING_FOR_STATE):
            return
        self.get_logger().warn("Interrupted, trying a safe landing")
        try:
            land_height = self.starting_position[2] if self.starting_position else 0
            self.cf.high_level_commander.land(absolute_height_m=land_height, duration_s=LAND_DURATION)
            time.sleep(LAND_DURATION + 0.5)
            self.cf.high_level_commander.stop()
            self.get_logger().warn("Emergency landing complete")
        except Exception as e:
            self.get_logger().error(f"Failed to emergency land {e}")
    

    def step_sequence(self):
        if self.sequence == QuadcopterSequence.WAITING_FOR_STATE:
            if self.got_first_state:
                self.get_logger().info("Got first state, now resetting estimator")
                self.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.05)
                self.cf.param.set_value('kalman.resetEstimation', '0')

                self.enter_state(QuadcopterSequence.STARTING_ESTIMATOR)
            elif self.time_elapsed() > 30:
                self.get_logger().error("No quadcopter state received in time, aborting...")
                self.shutdown()
        elif self.sequence == QuadcopterSequence.STARTING_ESTIMATOR:
            if self.time_elapsed() > 2:
                self.get_logger().info("Arming...")
                self.cf.supervisor.send_arming_request(True)
                self.enter_state(QuadcopterSequence.ARMING)
        elif self.sequence == QuadcopterSequence.ARMING:
            if self.time_elapsed() > 5:
                self.get_logger().info("Taking off...")
                self.cf.high_level_commander.takeoff(self.target_z, HOVER_DURATION)
                self.enter_state(QuadcopterSequence.TAKEOFF)
        elif self.sequence == QuadcopterSequence.TAKEOFF:
            if self.time_elapsed() > HOVER_DURATION + 0.5:
                self.get_logger().info(f"Beginning hover at {self.target_z}m for {HOVER_DURATION}s")
                self.cf.high_level_commander.go_to(x=self.starting_position[0], y=self.starting_position[1], z=self.target_z, yaw=0, duration_s=HOVER_DURATION)
                self.enter_state(QuadcopterSequence.HOVERING)
        elif self.sequence == QuadcopterSequence.HOVERING:
            if self.time_elapsed() > HOVER_DURATION:
                self.get_logger().info(f"Landing...")
                self.cf.high_level_commander.land(absolute_height_m=self.starting_position[2], duration_s=LAND_DURATION)
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
    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt received")
        node.emergency_land()
    except Exception as e:
        node.get_logger().warn(f"ERROR: {e}")
        node.emergency_land()
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

(In WSL)
lsusb
sudo chmod 666 /dev/bus/usb/001/<ID from lsusb>

colcon build --packages-select thesis_optitrack_bridge
source install/setup.bash
ros2 run thesis_optitrack_bridge hover_test_node

(when done)
usbipd detach --busid <BUS_ID>
'''