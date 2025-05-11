# ros2_motor_driver_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from can_msgs.msg import Frame
from std_msgs.msg import Empty
import can
from time import sleep

BASE_WIDTH = 353.5  # mm
ACCEL_MAX = 1000  # mm / s^2
# CAN_OUT_TOPIC = "/CAN/can0/transmit"
# CAN_IN_TOPIC = "/CAN/can0/receive"
CAN_INTERFACE = 'can0'


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
        self.get_logger().info("Initializing...")

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # self.publisher = self.create_publisher(Frame, CAN_OUT_TOPIC, 10)

        self.left_id = 0x601
        self.right_id = 0x602
        self.left_resp = 0x581
        self.right_resp = 0x582

        self.start_motors()
        self.get_logger().info("Motors started.")

    # def create_frame(self, arbitration_id, data):
    #     frame = Frame()
    #     frame.id = arbitration_id
    #     frame.is_extended = False
    #     frame.dlc = len(data)
    #     frame.data = data + [0] * (8 - len(data))  # Ensure 8 bytes
    #     return frame

    def send_command(self, motor, data):
        arbitration_id = self.left_id if motor == 'left' else self.right_id

        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        res = self.bus.recv(timeout=1.0)
        return res

    def start_motors(self):
        speed_bytes = list((ACCEL_MAX).to_bytes(4, byteorder='little', signed=True))
        commands = [
            [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00],
            [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00],
            [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00],
            None,
            [0x23, 0x83, 0x60, 0x00] + speed_bytes,
            [0x23, 0x84, 0x60, 0x00] + speed_bytes,
            [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00],
            None,
        ]
        for cmd in commands:
            if cmd is None:
                sleep(0.5)
            else:
                self.send_command('left', cmd)
                self.send_command('right', cmd)

    def stop_motors(self):
        stop_cmd = [0x2B, 0x40, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00]
        self.send_command('left', stop_cmd)
        self.send_command('right', stop_cmd)

    def set_speed(self, motor, speed):
        inverse = motor == 'right'
        speed_limit = 3000
        if inverse:
            speed = -speed
        speed = min(max(speed, -speed_limit), speed_limit)
        speed_bytes = list(speed.to_bytes(4, byteorder='little', signed=True))

        cmds = [
            [0x23, 0xFF, 0x60, 0x00] + speed_bytes,
            [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00],
        ]
        for cmd in cmds:
            self.send_command(motor, cmd)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x * 1000.0  # convert m/s to mm/s
        angular = msg.angular.z
        left_speed = int(linear - (BASE_WIDTH / 2.0) * angular)
        right_speed = int(linear + (BASE_WIDTH / 2.0) * angular)

        self.set_speed('left', left_speed)
        self.set_speed('right', right_speed)

    def destroy_node(self):
        self.set_speed('left', 0)
        self.set_speed('right', 0)
        self.stop_motors()
        self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
