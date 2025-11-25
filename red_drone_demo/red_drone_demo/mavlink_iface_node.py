import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil
import time

class MavlinkIface(Node):
    def __init__(self):
        super().__init__('mavlink_iface')
        self.declare_parameter('conn_str', 'udp:127.0.0.1:14550')
        self.conn_str = self.get_parameter('conn_str').get_parameter_value().string_value
        self.create_subscription(String, '/mavlink/cmd', self.cmd_cb, 10)

        self.get_logger().info(f'Connecting to MAVLink at {self.conn_str}')
        self.master = mavutil.mavlink_connection(self.conn_str)
        self.master.wait_heartbeat()
        self.get_logger().info('Heartbeat received')

    def set_mode_guided(self):
        self.master.set_mode_apm('GUIDED')
        self.get_logger().info('Mode GUIDED requested')

    def arm(self):
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        self.get_logger().info('Armed')

    def disarm(self):
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        self.get_logger().info('Disarmed')

    def takeoff(self, alt=10.0):
        # ArduPilot MAVProxy eşdeğeri: mode guided; arm; takeoff alt
        self.set_mode_guided()
        self.arm()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, alt
        )
        self.get_logger().info(f'Takeoff command sent to {alt} m')

    def land(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0
        )
        self.get_logger().info('Land command sent')

    def cmd_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'Received CMD: {cmd}')
        try:
            if cmd.startswith('TAKEOFF'):
                parts = cmd.split()
                alt = float(parts[1]) if len(parts) > 1 else 10.0
                self.takeoff(alt)
            elif cmd == 'LAND':
                self.land()
            elif cmd == 'ARM':
                self.arm()
            elif cmd == 'DISARM':
                self.disarm()
            elif cmd == 'GUIDED':
                self.set_mode_guided()
            else:
                self.get_logger().warn(f'Unknown command: {cmd}')
        except Exception as e:
            self.get_logger().error(f'CMD error: {e}')

def main():
    rclpy.init()
    node = MavlinkIface()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()