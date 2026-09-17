import time
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from math import sqrt, degrees, radians

from thesis_interfaces.msg import QuadcopterState, ControllerMode, MPCCommand, PlatformState

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import csv
from cflib.crazyflie.log import LogConfig
from datetime import datetime
from pathlib import Path


RADIO_URI = "radio://0/80/2M/E7E7E7E7E7"
TAKEOFF_DURATION = 2.0
LAND_DURATION = 3
MAX_POSSIBLE_SPEED = 3


class QuadcopterSequence(Enum):
    WAITING_FOR_STATE = auto()
    STARTING_ESTIMATOR = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVERING = auto()
    HANDOVER = auto()
    MPC_ACTIVE = auto()
    LANDING = auto()
    DONE = auto()


class FlightControlNode(Node):
    def __init__(self):
        super().__init__('flight_control_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        mpc_active_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.got_first_state = False
        self.starting_position = []
        self.sequence = QuadcopterSequence.WAITING_FOR_STATE
        self.state_entered_at = self.get_clock().now()
        self.target_z = None
        self.last_valid_pos = None
        self.last_valid_att = None
        self.last_valid_time = None

        self.last_msg_time = None
        self.max_gap_seen = 0
        self.gap_count = 0
        self.emergency_stopped = False
        self._hover_commanded = False
        self.rejected_count = 0
        self.consecutive_rejects = 0

        self.latest_mpc_setpoint = None
        self.latest_mpc_cmd_time = None

        self.declare_parameter('TRACK_HEIGHT', 1.0)
        self.TRACK_HEIGHT = float(self.get_parameter('TRACK_HEIGHT').value)
        self.declare_parameter('MAX_HEIGHT', 2.0)
        self.MAX_HEIGHT = float(self.get_parameter('MAX_HEIGHT').value)
        self.declare_parameter('MAX_HORIZONTAL_DISPLACEMENT', 1.0)
        self.MAX_HORIZONTAL_DISPLACEMENT = float(self.get_parameter('MAX_HORIZONTAL_DISPLACEMENT').value)
        self.declare_parameter('MAX_ATTITUDE_DEG', 25.0)
        self.MAX_ATTITUDE_DEG = float(self.get_parameter('MAX_ATTITUDE_DEG').value)
        self.declare_parameter('HOVER_DURATION', 3.0)
        self.HOVER_DURATION = float(self.get_parameter('HOVER_DURATION').value)
        self.declare_parameter('HANDOVER_DURATION', 2.0)
        self.HANDOVER_DURATION = float(self.get_parameter('HANDOVER_DURATION').value)

        self.declare_parameter('CRAZYFLIE_HOVER_THRUST', 39250)
        self.CRAZYFLIE_HOVER_THRUST = int(self.get_parameter('CRAZYFLIE_HOVER_THRUST').value)
        self.declare_parameter('CRAZYFLIE_HOVER_CONSTANT', 102040) # specifically 1 / (9.8e-6)
        self.CRAZYFLIE_HOVER_CONSTANT = int(self.get_parameter('CRAZYFLIE_HOVER_CONSTANT').value)

        cflib.crtp.init_drivers()
        self.get_logger().info("Connecting to CrazyFlie...")
        self.sync_cf = SyncCrazyflie(RADIO_URI, cf=Crazyflie(rw_cache='./cache'))
        self.sync_cf.open_link()

        self.cf = self.sync_cf.cf
        self.get_logger().info("Radio link set up")
        self.cf.param.set_value('stabilizer.estimator', '2')

        self.timer = self.create_timer(0.05, self.step_sequence)
        self.get_logger().info("Starting hover, waiting for state estimate")
        self.platform_pos = None

        self.quadcopter_state_subscription = self.create_subscription(
            QuadcopterState, '/measured_quadcopter_state', self.sequence_callback, qos
        )
        
        self.platform_state_subscription = self.create_subscription(
            PlatformState, '/full_platform_state', self.platform_callback, qos
        )

        self.controller_mode_publisher = self.create_publisher(
            ControllerMode, '/controller_mode', mpc_active_qos
        )

        self.mpc_cmd_subscription = self.create_subscription(
            MPCCommand, '/mpc_cmd', self.mpc_cmd_callback, 10
        )

        log_dir = Path.home() / "preliminary_mpc_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f"flight_control_{timestamp}.csv"

        self.log_file = open(log_path, "w", newline="", buffering=1)
        self.log_writer = csv.writer(self.log_file)

        self.log_writer.writerow([
            "time", "sequence", "x", "y", "z",
            "phi_measured", "theta_measured", 
            "sent_phi_deg", "sent_theta_deg", "sent_thrust_raw",
            "state_age", "mpc_cmd_age", "rejected_optitrack_count",
        ])

        self.get_logger().info(f"Logging flight data to {log_path}")
        self.last_sent_phi = 0.0
        self.last_sent_theta = 0.0
        self.last_sent_thrust = 0

    def log_flight_data(self):
        now = self.get_clock().now()
        if self.last_valid_pos is not None:
            x, y, z = self.last_valid_pos
        else:
            x = y = z = float("nan")

        if self.last_valid_att is not None:
            phi, theta = self.last_valid_att
        else:
            phi = theta = float("nan")

        if self.last_valid_time is not None:
            state_age = (now - self.last_valid_time).nanoseconds * 1e-9
        else:
            state_age = float("nan")

        if self.latest_mpc_cmd_time is not None:
            mpc_cmd_age = (now - self.latest_mpc_cmd_time).nanoseconds * 1e-9
        else:
            mpc_cmd_age = float("nan")

        self.log_writer.writerow([
            now.nanoseconds * 1e-9,
            self.sequence.name, x, y, z,
            phi, theta, self.last_sent_phi,
            self.last_sent_theta, self.last_sent_thrust,
            state_age, mpc_cmd_age, self.rejected_count,
        ])

    def platform_callback(self, msg: PlatformState):
        self.platform_pos = list(msg.position)


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
        self.last_valid_att = (msg.attitude[0], msg.attitude[1])
        self.last_valid_time = now
        try:
            self.cf.extpos.send_extpos(x, y, z)
        except Exception as e:
            self.get_logger().error(f"Failed to send extpos: {e}")


    def mpc_cmd_callback(self, msg: MPCCommand):
        if self.sequence != QuadcopterSequence.MPC_ACTIVE:
            return

        phi_cmd = degrees(msg.phi_cmd)
        theta_cmd = degrees(msg.theta_cmd)
        thrust_dev = msg.thrust_dev
        thrust_cmd = self.CRAZYFLIE_HOVER_THRUST + self.CRAZYFLIE_HOVER_CONSTANT * thrust_dev
        thrust_cmd = int(round(thrust_cmd))
        thrust_cmd = max(10001, min(60000, thrust_cmd)) # crazyflie limits

        self.latest_mpc_setpoint = (phi_cmd, theta_cmd, 0.0, thrust_cmd)
        self.latest_mpc_cmd_time = self.get_clock().now()


    def check_safety_violation(self):
        if self.last_valid_pos is None:
            return "No valid quadcopter position"

        x, y, z = self.last_valid_pos
        if z > self.MAX_HEIGHT:
            return f"Height {z} exceeds {self.MAX_HEIGHT}m limit"

        if self.starting_position:
            dx = x - self.starting_position[0]
            dy = y - self.starting_position[1]
            horizontal_distance = sqrt(dx ** 2 + dy**2)
            if horizontal_distance > self.MAX_HORIZONTAL_DISPLACEMENT:
                return f"Horizontal displacement {horizontal_distance:.2f} exceeds {self.MAX_HORIZONTAL_DISPLACEMENT}m limit"
        if self.last_valid_att is not None:
            phi, theta = self.last_valid_att
            max_att = radians(self.MAX_ATTITUDE_DEG)
            if abs(phi) > max_att or abs(theta) > max_att:
                return f"Excessive attitude: phi={degrees(phi):.1f}deg, theta={degrees(theta):.1f}deg"
        return None


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


    def emergency_land(self, message=''):
        if self.sequence in (QuadcopterSequence.DONE, QuadcopterSequence.WAITING_FOR_STATE):
            return
        self.get_logger().warn(f"Emergency Landing: {message}")
        controller_mode = ControllerMode()
        controller_mode.header.stamp = self.get_clock().now().to_msg()
        controller_mode.mode = ControllerMode.TRACKING_MODE
        controller_mode.mpc_active = ControllerMode.MPC_INACTIVE
        controller_mode.t_start_land = 0.0
        controller_mode.z_start_land = 0.0
        self.controller_mode_publisher.publish(controller_mode)

        try: 
            self.cf.commander.send_notify_setpoint_stop()
            land_height = self.starting_position[2] if self.starting_position else 0.0
            self.cf.high_level_commander.land(absolute_height_m=land_height, duration_s=LAND_DURATION)
            self.enter_state(QuadcopterSequence.LANDING)
            self.get_logger().warn("Emergency landing complete")
        except Exception as e:
            self.get_logger().error(f"Failed to emergency land {e}")
    
    
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


    def step_sequence(self):
        if self.emergency_stopped:
            return
        self.log_flight_data()
        if self.sequence == QuadcopterSequence.WAITING_FOR_STATE:
            if self.got_first_state and self.platform_pos is not None:
                self.target_z = self.platform_pos[2] + self.TRACK_HEIGHT
                self.reset_internal_kalman()
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
                self.cf.high_level_commander.takeoff(
                    absolute_height_m=self.target_z, 
                    duration_s=TAKEOFF_DURATION,
                    yaw=None)
                self.enter_state(QuadcopterSequence.TAKEOFF)

        elif self.sequence == QuadcopterSequence.TAKEOFF:
            if self.time_elapsed() > TAKEOFF_DURATION + 0.5:
                if not self._hover_commanded:
                    self.get_logger().info(f"Beginning hover at {self.target_z}m for {self.HOVER_DURATION}s")
                    self.cf.high_level_commander.go_to(x=self.starting_position[0], y=self.starting_position[1], z=self.target_z, yaw=0, duration_s=1.0)
                    self._hover_commanded = True
                if self.time_elapsed() > TAKEOFF_DURATION + 1.5:
                    self.get_logger().info(f"Hovering for {self.HOVER_DURATION}s")
                    self.enter_state(QuadcopterSequence.HOVERING)

        elif self.sequence == QuadcopterSequence.HOVERING:
            if self.time_elapsed() > self.HOVER_DURATION:
                self.get_logger().info(f"Handing over to low level control...")
                self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                self.cf.commander.send_setpoint(0.0, 0.0, 0.0, self.CRAZYFLIE_HOVER_THRUST)
                self.enter_state(QuadcopterSequence.HANDOVER)

        elif self.sequence == QuadcopterSequence.HANDOVER:
            self.cf.commander.send_setpoint(0.0, 0.0, 0.0, self.CRAZYFLIE_HOVER_THRUST)
            if self.time_elapsed() > self.HANDOVER_DURATION:
                self.get_logger().info(f"Transitioning to MPC control... ")

                controller_mode = ControllerMode()
                controller_mode.header.stamp = self.get_clock().now().to_msg()
                controller_mode.mode = ControllerMode.TRACKING_MODE
                controller_mode.mpc_active = ControllerMode.MPC_ACTIVE
                controller_mode.t_start_land = 0.0
                controller_mode.z_start_land = 0.0
                self.controller_mode_publisher.publish(controller_mode)
                self.latest_mpc_setpoint = None
                self.latest_mpc_cmd_time = None
                self.enter_state(QuadcopterSequence.MPC_ACTIVE)
            
        elif self.sequence == QuadcopterSequence.MPC_ACTIVE:
            if self.last_valid_time is None:
                self.emergency_land("No valid quadcopter state received")
                return

            state_age = (self.get_clock().now() - self.last_valid_time).nanoseconds * 1e-9
            if state_age > 0.2:
                self.emergency_land("Quadcopter state stale for more than 0.2s")
                return
            
            # violation = self.check_safety_violation()
            # if violation  is not None:
            #     self.emergency_land(violation)
            #     return
            
            if self.latest_mpc_cmd_time is None or self.latest_mpc_setpoint is None:
                self.cf.commander.send_setpoint(0.0, 0.0, 0.0, self.CRAZYFLIE_HOVER_THRUST)
                if self.time_elapsed() > 1.0:
                    self.emergency_land("No MPC commands received")
                return

            cmd_age = (self.get_clock().now() - self.latest_mpc_cmd_time).nanoseconds * 1e-9
            if cmd_age > 1.0:
                self.emergency_land("No MPC commands received 1s, landing")
                return
            elif cmd_age > 0.2:
                self.cf.commander.send_setpoint(0.0, 0.0, 0.0, self.CRAZYFLIE_HOVER_THRUST)
                return

            #normal mpc operation
            phi, theta, yaw_rate, thrust = self.latest_mpc_setpoint
            self.cf.commander.send_setpoint(phi, theta, yaw_rate, thrust)
            

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
    node = FlightControlNode()

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
            node.log_file.close()
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