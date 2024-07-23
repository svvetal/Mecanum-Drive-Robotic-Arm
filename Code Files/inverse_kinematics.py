import rclpy
from rclpy.node import Node
import math
import time

from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
servo = 16

class ServoControl(Node):
    def __init__(self):
        super().__init__("servo_control")

        self.initial_base = 65
        self.initial_a1 = 165
        self.initial_a2 = 180
        self.upp_angle = 85
        self.grip_angle = 100

        kit.servo[14].angle = int(self.initial_base)
        kit.servo[8].angle = int(self.initial_a1)
        kit.servo[2].angle = int(self.initial_a2)
        kit.servo[5].angle = int(self.upp_angle)
        kit.servo[11].angle = int(self.grip_angle)

        time.sleep(3)

    def goal_to_reach(self, x, y, z):
        base = math.degrees(math.atan2(y,x))

        l = math.sqrt(pow(x,2) + pow(y,2))

        phi = math.degrees(math.atan2(z,l))

        h = math.sqrt(pow(l,2) + pow(z,2))

        theta = math.degrees(math.acos((h/2)/140))

        final_a1 = phi + theta
        final_a2 = phi - theta
        final_base = 90 + base

        final_a1 = int(final_a1)
        final_a2 = int(120 - final_a2)
        final_base = int(final_base)

        print("final_angles")
        print(final_base, final_a1, final_a2)
        return final_base, final_a1, final_a2

    def actuate_servos(self, final_base, final_a1, final_a2):
        if(self.initial_base < final_base):
            for i in range(self.initial_base, final_base+1, 1):
                kit.servo[14].angle = int(i)

        elif(self.initial_base > final_base):
            for i in range(self.initial_base, final_base-1, -1):
                kit.servo[14].angle = int(i)

        if (self.initial_a1 > final_a1):
            for i in range(self.initial_a1, final_a1-1, -1):
                kit.servo[2].angle = int(i)

        if (self.initial_a2 > final_a2):
            for i in range(self.initial_a2, final_a2-1, -1):
                kit.servo[5].angle = int(i)

    def gripper_grab(self):
        for i in range(self.grip_angle, -1, -1):
                print(i)
                kit.servo[11].angle = int(i)

    def gripper_release(self):
        for i in range(0, self.grip_angle+1, 1):
                print(i)
                kit.servo[11].angle = int(i)

    def wrist_down(self):
        for i in range(self.upp_angle, -1, -1):
                print(i)
                kit.servo[8].angle = int(i)

    def wrist_up(self):
        for i in range(0, self.upp_angle, 1):
                print(i)
                kit.servo[8].angle = int(i)

def main():
    rclpy.init()
    servo_control = ServoControl()
    final_base , final_a1 , final_a2 = servo_control.goal_to_reach(100,100,50)
    servo_control.actuate_servos(final_base, final_a1 , final_a2)
    time.sleep(2)
    servo_control.gripper_grab()
    time.sleep(2)
    servo_control.wrist_down()
    time.sleep(3)
    servo_control.wrist_up()
    time.sleep(3)
    rclpy.spin(servo_control)
    servo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
