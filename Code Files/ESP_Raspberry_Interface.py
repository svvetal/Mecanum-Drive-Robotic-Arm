import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import serial
import time
import struct

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

class LocControl(Node):
    def __init__(self):
        super().__init__("loc_control")

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def ps4_deadband(self, ps4_value):
        if((ps4_value < 5) and (ps4_value > -5)):
            ps4_value = 0
        return ps4_value

    def send_data(self, values):
        data = struct.pack('fff', *values)

        ser.write(data)

    def joy_callback(self, msg:Joy):

        Right_Y = self.ps4_deadband(msg.axes[4] * 255)
        Right_X = self.ps4_deadband(msg.axes[3] * 255)
        Left_X = self.ps4_deadband(msg.axes[0] * 255)

        floats = [Right_Y, Right_X, Left_X]

        self.send_data(floats)
        print(ser.readline().decode().strip())


def main():
    rclpy.init()
    loc_control = LocControl()
    rclpy.spin(loc_control)
    loc_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()