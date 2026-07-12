import csv
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from thesis_interfaces.msg import QuadcopterState

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


RADIO_URI = "radio://0/80/2M/E7E7E7E7E7"

TARGET_HEIGHT = 0.5
HOVER_DURATION = 5
STEP_DURATION = 3
RECOVER_DURATION = 2
LAND_DURATION = 3

ROLL_STEP_DEG = 15
PITCH_STEP_DEG = 0

LOG_PERIOD_MS = 20
LOG_FILENAME = f"step_response_{time.strftime('%Y%m%d_%H%M%S')}.csv"

MAX_STEP_DEG = 20


class QuadcopterSequence(Enum):
    WAITING_FOR_STATE = auto()
    STARTING_ESTIMATOR = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVER = auto()
    STEP = auto()
    RECOVER = auto()
    LANDING = auto()
    DONE = auto()


class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        assert abs(ROLL_STEP_DEG) <= MAX_STEP_DEG
        assert abs(PITCH_STEP_DEG) <= MAX_STEP_DEG

        self.got_first_state = False
        self.sequence = QuadcopterSequence.WAITING_FOR_STATE
        self.state_entered_at = self.get_clock().now()

        self.subscription = self.create_subscription(
            QuadcopterState, '/quadcopter_state', self.state_callback, qos
        )

        cflib.crtp.init_drivers()
        self.get_logger().info("Connecting to CrazyFlie...")
        self.sync_cf = SyncCrazyflie(RADIO_URI, cf=Crazyflie(rw_cache='./cache'))
        self.sync_cf.open_link()
        self.cf = self.sync_cf.cf
        self.get_logger().info("Radio link set up")

        self.log_rows = []
        self._setup_attitude_logging()

        self.timer = self.create_timer(0.1, self.step_sequence)
        self.get_logger().info("Starting motion test, waiting for state estimate")

    def _setup_attitude_logging(self):
        log_conf = LogConfig(name='Attitude', period_in_ms=LOG_PERIOD_MS)
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')
        log_conf.add_variable('stabilizer.yaw', 'float')
        self.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self._attitude_log_callback)
        log_conf.start()
        self._log_conf = log_conf

    def _attitude_log_callback(self, timestamp, data, logconf):
        self.log_rows.append({
            'timestamp_ms': timestamp,
            'wall_time': time.time(),
            'sequence_state': self.sequence.name,
            'roll': data['stabilizer.roll'],
            'pitch': data['stabilizer.pitch'],
            'yaw': data['stabilizer.yaw'],
        })

    def _write_log_csv(self):
        if not self.log_rows:
            self.get_logger().warning("No attitude log rows captured, skipping CSV write")
            return
        fieldnames = list(self.log_rows[0].keys())
        with open(LOG_FILENAME, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.log_rows)
        self.get_logger().info(f"Wrote {len(self.log_rows)} log rows to {LOG_FILENAME}")


    def state_callback(self, msg: QuadcopterState):
        x, y, z = msg.position[0], msg.position[1], msg.position[2]
        self.got_first_state = True
        try:
            self.cf.extpos.send_extpos(x, y, z)
        except Exception as e:
            self.get_logger().error(f"Failed to send extpos: {e}")

    def enter_state(self, new_state: QuadcopterSequence):
        self.get_logger().info(f"State transition: {self.sequence.name} -> {new_state.name}")
        self.sequence = new_state
        self.state_entered_at = self.get_clock().now()

    def time_elapsed(self):
        return (self.get_clock().now() - self.state_entered_at).nanoseconds / 1e9

    def shutdown(self):
        self.timer.cancel()
        try:
            self._log_conf.stop()
        except Exception:
            pass
        self._write_log_csv()
        try:
            self.sync_cf.close_link()
        except Exception:
            pass

    def step_sequence(self):
        s = self.sequence

        if s == QuadcopterSequence.WAITING_FOR_STATE:
            if self.got_first_state:
                self.get_logger().info("Got first state, resetting estimator")
                self.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.05)
                self.cf.param.set_value('kalman.resetEstimation', '0')
                self.enter_state(QuadcopterSequence.STARTING_ESTIMATOR)
            elif self.time_elapsed() > 15:
                self.get_logger().error("No quadcopter state received in time, aborting")
                self.shutdown()

        elif s == QuadcopterSequence.STARTING_ESTIMATOR:
            if self.time_elapsed() > 2:
                self.get_logger().info("Arming...")
                self.cf.platform.send_arming_request(True)
                self.enter_state(QuadcopterSequence.ARMING)

        elif s == QuadcopterSequence.ARMING:
            if self.time_elapsed() > 1:
                self.get_logger().info("Taking off...")
                self.cf.high_level_commander.takeoff(TARGET_HEIGHT, HOVER_DURATION)
                self.enter_state(QuadcopterSequence.TAKEOFF)

        elif s == QuadcopterSequence.TAKEOFF:
            if self.time_elapsed() > HOVER_DURATION + 0.5:
                self.get_logger().info(f"Holding hover at {TARGET_HEIGHT} m")
                self.enter_state(QuadcopterSequence.HOVER)

        elif s == QuadcopterSequence.HOVER:
            # Just settling here - no HL commands needed, it holds position.
            if self.time_elapsed() > HOVER_DURATION:
                self.get_logger().info(
                    f"Commanding step: roll={ROLL_STEP_DEG} deg, pitch={PITCH_STEP_DEG} deg"
                )
                self.enter_state(QuadcopterSequence.STEP)

        elif s == QuadcopterSequence.STEP:
            self.cf.commander.send_zdistance_setpoint(
                ROLL_STEP_DEG, PITCH_STEP_DEG, 0, TARGET_HEIGHT
            )
            if self.time_elapsed() > STEP_DURATION:
                self.enter_state(QuadcopterSequence.RECOVER)

        elif s == QuadcopterSequence.RECOVER:
            self.cf.commander.send_zdistance_setpoint(0, 0, 0, TARGET_HEIGHT)
            if self.time_elapsed() > RECOVER_DURATION:
                self.get_logger().info("Landing...")
                self.cf.high_level_commander.land(absolute_height_m=0, duration_s=LAND_DURATION)
                self.enter_state(QuadcopterSequence.LANDING)

        elif s == QuadcopterSequence.LANDING:
            if self.time_elapsed() > LAND_DURATION + 0.5:
                self.cf.high_level_commander.stop()
                self.get_logger().info("Landed")
                self.enter_state(QuadcopterSequence.DONE)
                self.shutdown()

        elif s == QuadcopterSequence.DONE:
            pass


def main():
    rclpy.init()
    node = MotionTestNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        try:
            node.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
colcon build --packages-select thesis_optitrack_bridge
ros2 run thesis_optitrack_bridge motion_test_node
'''