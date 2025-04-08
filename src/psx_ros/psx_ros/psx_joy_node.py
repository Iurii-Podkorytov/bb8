import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import spidev
import time

class PSXJoyNode(Node):
    def __init__(self):
        super().__init__('ps2_joy_node')
        
        # ROS2 publisher
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        # SPI setup
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # SPI0 CE0
        self.spi.max_speed_hz = 250000  # Adjust if needed
        
        # PS2 controller configuration
        self.configure_analog_mode()
        
        # Poll command for analog mode (from PsxNewLib)
        self.poll_cmd = [0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        # Button mapping (adapted from PsxNewLib)
        self.button_map = {
            'select': 0b00000001,
            'l3':     0b00000010,
            'r3':     0b00000100,
            'start':  0b00001000,
            'up':     0b00010000,
            'right':  0b00100000,
            'down':   0b01000000,
            'left':   0b10000000,
            'l2':     0b00000001,
            'r2':     0b00000010,
            'l1':     0b00000100,
            'r1':     0b00001000,
            'triangle': 0b00010000,
            'circle':  0b00100000,
            'cross':   0b01000000,
            'square':  0b10000000
        }

    def configure_analog_mode(self):
        """Configure controller for analog mode (adapted from PsxNewLib)"""
        # Enter config mode
        self.spi.xfer2([0x43, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00])
        # Enable analog mode
        self.spi.xfer2([0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        # Exit config mode
        self.spi.xfer2([0x43, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00])

    def read_controller(self):
        """Read controller data (based on PsxNewLib's read() function)"""
        response = self.spi.xfer2(self.poll_cmd)
        
        if len(response) < 9:
            self.get_logger().warn("Invalid response length")
            return None
        
        # Parse button data (bytes 3-8)
        buttons = ((response[4] << 8) | response[3])  # Combine 2 bytes
        
        # Check protocol type (adapted from PsxNewLib's logic)
        if len(response) >= 21 and (response[1] & 0xF0) == 0x70:
            # DualShock2 analog mode
            analog = {
                'lx': response[7],
                'ly': response[8],
                'rx': response[5],
                'ry': response[6],
                'pressure': response[9:21]  # Pressure-sensitive buttons
            }
        else:
            # Digital mode or basic analog
            analog = {
                'lx': 128,
                'ly': 128,
                'rx': 128,
                'ry': 128
            }
        
        return {'buttons': buttons, 'analog': analog}

    def timer_callback(self):
        data = self.read_controller()
        if not data:
            return
        
        # Create Joy message
        msg = Joy()
        
        # Map buttons (16 standard buttons)
        msg.buttons = [
            (data['buttons'] & self.button_map['select']) >> 0,
            (data['buttons'] & self.button_map['l3']) >> 1,
            (data['buttons'] & self.button_map['r3']) >> 2,
            (data['buttons'] & self.button_map['start']) >> 3,
            (data['buttons'] & self.button_map['up']) >> 4,
            (data['buttons'] & self.button_map['right']) >> 5,
            (data['buttons'] & self.button_map['down']) >> 6,
            (data['buttons'] & self.button_map['left']) >> 7,
            (data['buttons'] & self.button_map['l2']) >> 8,
            (data['buttons'] & self.button_map['r2']) >> 9,
            (data['buttons'] & self.button_map['l1']) >> 10,
            (data['buttons'] & self.button_map['r1']) >> 11,
            (data['buttons'] & self.button_map['triangle']) >> 12,
            (data['buttons'] & self.button_map['circle']) >> 13,
            (data['buttons'] & self.button_map['cross']) >> 14,
            (data['buttons'] & self.button_map['square']) >> 15
        ]
        
        # Map analog axes (0-255 → -1.0 to 1.0)
        msg.axes = [
            (data['analog']['lx'] - 128) / 128.0,
            (data['analog']['ly'] - 128) / 128.0,
            (data['analog']['rx'] - 128) / 128.0,
            (data['analog']['ry'] - 128) / 128.0
        ]
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PSXJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
