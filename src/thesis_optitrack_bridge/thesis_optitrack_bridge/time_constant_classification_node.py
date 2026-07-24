import csv
import time
from enum import Enum, auto
from math import sqrt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from thesis_interfaces.msg import QuadcopterState

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


RADIO_URI = "radio://0/80/2M/E7E7E7E7E7"
TAKEOFF_DURATION = 2.0
LAND_DURATION = 3
MAX_POSSIBLE_SPEED = 3

RECOVER_DURATION = 2
MAX_STEP_DEG = 20

LOG_PERIOD_MS = 20



class QuadcopterSequence(Enum):
    WAITING_FOR_STATE = auto()
    STARTING_ESTIMATOR = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVERING = auto()
    STEP = auto()
    RETURN = auto()
    RECOVER = auto()
    LANDING = auto()
    DONE = auto()


class TimeConstantClassificationNode(Node):
    def __init__(self):
        super().__init__('time_constant_classification_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.got_first_state = False
        self.starting_position = []
        self.sequence = QuadcopterSequence.WAITING_FOR_STATE
        self.state_entered_at = self.get_clock().now()
        self.target_z = None
        self.last_valid_pos = None
        self.last_valid_time = None
        self.rejected_count = 0
        self.consecutive_rejects = 0
        self._return_commanded = False

        self.last_msg_time = None
        self.max_gap_seen = 0
        self.gap_count = 0
        self.emergency_stopped = False
        self._hover_commanded = False
        self.subscription = self.create_subscription(
            QuadcopterState, '/measured_quadcopter_state', self.sequence_callback, qos
        )

        self.declare_parameter('TARGET_HEIGHT', 0.4)
        self.TARGET_HEIGHT = float(self.get_parameter('TARGET_HEIGHT').value)
        self.declare_parameter('MAX_HEIGHT', 2.0)
        self.MAX_HEIGHT = float(self.get_parameter('MAX_HEIGHT').value)
        self.declare_parameter('HOVER_DURATION', 5.0)
        self.HOVER_DURATION = float(self.get_parameter('HOVER_DURATION').value)
        self.declare_parameter('FLY_DURATION', 10.0)
        self.FLY_DURATION = float(self.get_parameter('FLY_DURATION').value)
        
        self.declare_parameter('ROLL_STEP_DEG', 5.0)
        self.ROLL_STEP_DEG = float(self.get_parameter('ROLL_STEP_DEG').value)
        self.declare_parameter('PITCH_STEP_DEG', 0.0)
        self.PITCH_STEP_DEG = float(self.get_parameter('PITCH_STEP_DEG').value)
        self.declare_parameter('STEP_DURATION', 2.0)
        self.STEP_DURATION = float(self.get_parameter('STEP_DURATION').value)

        assert abs(self.ROLL_STEP_DEG) <= MAX_STEP_DEG
        assert abs(self.PITCH_STEP_DEG) <= MAX_STEP_DEG
        
        self.LOG_FILENAME = f"log/step_response_{time.strftime('%Y%m%d_%H%M%S')}_r{int(self.ROLL_STEP_DEG)}p{int(self.PITCH_STEP_DEG)}_{int(self.STEP_DURATION)}s.csv"

        cflib.crtp.init_drivers()
        self.get_logger().info("Connecting to CrazyFlie...")
        self.sync_cf = SyncCrazyflie(RADIO_URI, cf=Crazyflie(rw_cache='./cache'))
        self.sync_cf.open_link()
        self.cf = self.sync_cf.cf
        self.get_logger().info("Radio link set up")
        self.cf.param.set_value('stabilizer.estimator', '2')

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
        with open(self.LOG_FILENAME, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.log_rows)
        self.get_logger().info(f"Wrote {len(self.log_rows)} log rows to {self.LOG_FILENAME}")


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

        if z > self.MAX_HEIGHT:
            self.get_logger().warn("Climbed too high, cutting power!")
            self.cf.high_level_commander.stop()
            self.cf.commander.send_stop_setpoint()
            self.emergency_stopped = True
            return
        
        if not self.got_first_state:
            self.get_logger().info(f"Got starting position {x}, {y}, {z}")
            self.starting_position = [x, y, z]
            self.target_z = z + self.TARGET_HEIGHT
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
                    self.consecutive_rejects += 1
                    self.get_logger().warn(f"Rejecting optitrack jump of {dist:.3f}m over {dt:.3f}s. Rejected counter: {self.rejected_count}, consecutive counter: {self.consecutive_rejects}")
                    if self.consecutive_rejects < 5:
                        return
                    self.get_logger().warn("5 consecutive rejects, treating as real motion and accepting")
                else:
                    self.consecutive_rejects = 0

        self.got_first_state = True
        self.last_valid_pos = (x, y, z)
        self.last_valid_time = now
        try:
            self.cf.extpos.send_extpos(x, y, z)
            t = now.nanoseconds / 1e9
        except Exception as e:
            self.get_logger().error(f"Failed to send extpos: {e}")


    def reset_internal_kalman(self):
            self.cf.param.set_value('kalman.initialX', str(self.starting_position[0]))
            self.cf.param.set_value('kalman.initialY', str(self.starting_position[1]))
            self.cf.param.set_value('kalman.initialZ', str(self.starting_position[2]))
            self.cf.param.set_value('kalman.initialYaw', '0.0')
            self.cf.param.set_value('stabilizer.controller', '1')
            self.get_logger().info("Got first state, now resetting estimator")
            self.cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            self.cf.param.set_value('kalman.resetEstimation', '0')


    def enter_state(self, new_state: QuadcopterSequence):
        self.get_logger().info(f"State transition: {self.sequence.name} -> {new_state.name}")
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
            self.cf.commander.send_notify_setpoint_stop()
            land_height = self.starting_position[2] if self.starting_position else 0
            self.cf.high_level_commander.land(absolute_height_m=land_height, duration_s=LAND_DURATION)
            time.sleep(LAND_DURATION + 0.5)
            self.cf.high_level_commander.stop()
            self.get_logger().warn("Emergency landing complete")
        except Exception as e:
            self.get_logger().error(f"Failed to emergency land {e}")


    def step_sequence(self):
        if self.emergency_stopped:
            return
        
        if self.sequence == QuadcopterSequence.WAITING_FOR_STATE:
            if self.got_first_state:
                self.reset_internal_kalman()
                self.enter_state(QuadcopterSequence.STARTING_ESTIMATOR)
            elif self.time_elapsed() > 15:
                self.get_logger().error("No quadcopter state received in time, aborting")
                self.shutdown()

        elif self.sequence == QuadcopterSequence.STARTING_ESTIMATOR:
            if self.time_elapsed() > 2:
                self.get_logger().info("Arming...")
                self.cf.supervisor.send_arming_request(True)
                self.enter_state(QuadcopterSequence.ARMING)

        elif self.sequence == QuadcopterSequence.ARMING:
            if self.time_elapsed() > 5:
                self.get_logger().info("Taking off...")
                self.cf.high_level_commander.takeoff(
                    absolute_height_m=self.target_z, 
                    duration_s=TAKEOFF_DURATION,
                    yaw=None)
                self.enter_state(QuadcopterSequence.TAKEOFF)

        elif self.sequence == QuadcopterSequence.TAKEOFF:
            if self.time_elapsed() > TAKEOFF_DURATION + 0.5:
                self.get_logger().info(f"Beginning hover at {self.target_z}m for {self.HOVER_DURATION}s")
                self.enter_state(QuadcopterSequence.HOVERING)

        elif self.sequence == QuadcopterSequence.HOVERING:
            if not self._hover_commanded:
                self.cf.high_level_commander.go_to(x=self.starting_position[0], y=self.starting_position[1], z=self.target_z, yaw=0, duration_s=self.HOVER_DURATION/2)
                self._hover_commanded = True
            if self.time_elapsed() > self.HOVER_DURATION:
                self.get_logger().info(
                    f"Commanding step: roll={self.ROLL_STEP_DEG} deg, pitch={self.PITCH_STEP_DEG} deg"
                )
                self.enter_state(QuadcopterSequence.STEP)

        elif self.sequence == QuadcopterSequence.STEP:
            t = self.time_elapsed()
            if t <= self.STEP_DURATION:
                self.cf.commander.send_zdistance_setpoint(
                    self.ROLL_STEP_DEG, self.PITCH_STEP_DEG, 0, self.target_z
                )
            else:
                self.enter_state(QuadcopterSequence.RETURN)
                
        elif self.sequence == QuadcopterSequence.RETURN:
            if not self._return_commanded:
                self.get_logger().info(f"Returing to initial hover...")
                self.cf.commander.send_notify_setpoint_stop()
                self.cf.high_level_commander.go_to(x=self.starting_position[0], y=self.starting_position[1], z=self.target_z, yaw=0, duration_s=RECOVER_DURATION)
                self._return_commanded = True
            elif self.time_elapsed() > RECOVER_DURATION + 1:
                self.enter_state(QuadcopterSequence.RECOVER)

        elif self.sequence == QuadcopterSequence.RECOVER:
            if self.time_elapsed() > RECOVER_DURATION:
                self.get_logger().info("Landing...")
                self.cf.high_level_commander.land(absolute_height_m=self.starting_position[2], duration_s=LAND_DURATION)
                self.enter_state(QuadcopterSequence.LANDING)

        elif self.sequence == QuadcopterSequence.LANDING:
            if self.time_elapsed() > LAND_DURATION + 0.5:
                self.cf.high_level_commander.stop()
                self.get_logger().info("Landed")
                self.enter_state(QuadcopterSequence.DONE)
                self.timer.cancel()

        elif self.sequence == QuadcopterSequence.DONE:
            pass


def main():
    rclpy.init()
    node = TimeConstantClassificationNode()
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
            node._write_log_csv()
            node.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
colcon build --packages-select thesis_optitrack_bridge
ros2 run thesis_optitrack_bridge time_constant_classification_node
'''