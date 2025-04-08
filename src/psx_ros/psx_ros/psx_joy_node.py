import rclpy
from sensor_msgs.msg import Joy
import spidev

class PSXJoyNode:
    def __init__(self):
        self.node = rclpy.create_node('ps2_joy_node')
        self.publisher = self.node.create_publisher(Joy, 'joy', 10)
        self.timer = self.node.create_timer(0.1, self.timer_callback)

        # SPI Configuration
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 250000 

        # PS2 Command Sequence (from PsxNewLib)
        self.cmd = [0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    def read_controller(self):
        # Send command and read response (9 bytes)
        response = self.spi.xfer2(self.cmd)
        # Parse response (see PS2 protocol [[8]])
        buttons = response[3:9]
        return {
            'buttons': buttons,
            'analog': {
                'rx': response[9],
                'ry': response[10],
                'lx': response[11],
                'ly': response[12]
            }
        }

    def timer_callback(self):
        data = self.read_controller()
        # Map buttons to Joy message (example for 16 buttons)
        msg = Joy()
        msg.buttons = [  # Modify based on your controller's button mapping
            (data['buttons'][0] >> 0) & 1,  # Select
            (data['buttons'][0] >> 1) & 1,  # L3
            (data['buttons'][0] >> 2) & 1,  # R3
            (data['buttons'][0] >> 3) & 1,  # Start
            (data['buttons'][1] >> 4) & 1,  # D-Pad Up
            (data['buttons'][1] >> 5) & 1,  # D-Pad Right
            (data['buttons'][1] >> 6) & 1,  # D-Pad Down
            (data['buttons'][1] >> 7) & 1,  # D-Pad Left
            # Add remaining buttons...
        ]
        # Map analog axes (convert 0-255 to -1.0 to 1.0)
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