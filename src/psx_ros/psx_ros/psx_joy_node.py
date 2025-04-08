import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import spidev

class PSXJoyNode(Node):
    def __init__(self):
        super().__init__('ps2_joy_node')
        
        # Publisher and timer
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz for better responsiveness
        
        # SPI configuration
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # SPI0 CE0
        self.spi.max_speed_hz = 100000  # Lower speed for reliability
        
        # PS2 controller setup
        self.configure_controller()
        
        # Poll command for analog mode
        self.poll_cmd = [0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        # Button mapping (adapted for non-DualShock2 controllers)
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

    def configure_controller(self):
        """Force analog mode configuration"""
        # Enter config mode
        self.spi.xfer2([0x43, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00])
        # Enable analog mode
        self.spi.xfer2([0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        # Exit config mode
        self.spi.xfer2([0x43, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00])
        # Set vibration (some controllers require this)
        self.spi.xfer2([0x4D, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def read_controller(self):
        """Read controller data with improved parsing"""
        response = self.spi.xfer2(self.poll_cmd)
        
        if len(response) < 5:
            return None
        
        # Parse buttons (byte 3-4 for digital buttons)
        buttons = ((response[4] << 8) | response[3])  # 16-bit button data
        
        # Check for analog mode (even if not DualShock2)
        if len(response) >= 9:
            # Analog stick data starts at byte 5 (some controllers send 9 bytes)
            lx = response[5]
            ly = response[6]
            rx = response[7]
            ry = response[8]
        else:
            # Fallback to digital (though analog sticks should still work)
            lx = ly = rx = ry = 128

        self.get_logger().info(f"Dec: {list(response)}")
        
        return {
            'buttons': buttons,
            'analog': {
                'lx': lx,
                'ly': ly,
                'rx': rx,
                'ry': ry
            }
        }

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