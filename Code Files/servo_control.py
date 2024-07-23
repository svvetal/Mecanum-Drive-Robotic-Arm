import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
servo = 16

class ServoControl(Node):
    def __init__(self):
        super().__init__("servo_control")

        self.base_angle = 165
        self.mid_angle = 180
        self.upp_angle = 180
        self.grip_angle = 80
        self.rotate_angle = 40

        kit.servo[2].angle = int(self.base_angle)
        kit.servo[5].angle = int(self.mid_angle)
        kit.servo[8].angle = int(self.upp_angle)
        kit.servo[11].angle = int(self.grip_angle)
        kit.servo[14].angle = int(self.rotate_angle)

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def drive_base_servo(self, base_pos, base_neg):
        if(base_pos==1):
            self.base_angle = self.base_angle+5
            if (self.base_angle >= 165):
                self.base_angle = 165
                kit.servo[2].angle=int(self.base_angle)
            else:
                kit.servo[2].angle=int(self.base_angle)
        if(base_neg==1):
            self.base_angle = self.base_angle-5
            if (self.base_angle <= 0):
                self.base_angle = 0
                kit.servo[2].angle=int(self.base_angle)
            else:
                kit.servo[2].angle=int(self.base_angle)

    def drive_mid_servo(self, mid_pos, mid_neg):
        if(mid_pos==1):
            self.mid_angle = self.mid_angle+5
            if (self.mid_angle >= 180):
                self.mid_angle = 180
                kit.servo[5].angle=int(self.mid_angle)
            else:
                kit.servo[5].angle=int(self.mid_angle)
        if(mid_neg==1):
            self.mid_angle = self.mid_angle-5
            if (self.mid_angle <= 60):
                self.mid_angle = 60
                kit.servo[5].angle=int(self.mid_angle)
            else:
                kit.servo[5].angle=int(self.mid_angle)

    def drive_upp_servo(self, upp_pos, upp_neg):
        if(upp_pos==1):
            self.upp_angle = self.upp_angle+5
            if (self.upp_angle >= 180):
                self.upp_angle = 180
                kit.servo[8].angle=int(self.upp_angle)
            else:
                kit.servo[8].angle=int(self.upp_angle)
        if(upp_neg==1):
            self.upp_angle = self.upp_angle-5
            if (self.upp_angle <= 100):
                self.upp_angle = 100
                kit.servo[8].angle=int(self.upp_angle)
            else:
                kit.servo[8].angle=int(self.upp_angle)

    def grip_servo(self, grip):
        if(grip==1):
            self.grip_angle = self.grip_angle+5
            if (self.grip_angle >= 80):
                self.grip_angle = 80
                kit.servo[11].angle=int(self.grip_angle)
            else:
                kit.servo[11].angle=int(self.grip_angle)
        if(grip==(-1)):
            self.grip_angle = self.grip_angle-5
            if (self.grip_angle <= 0):
                self.grip_angle = 0
                kit.servo[11].angle=int(self.grip_angle)
            else:
                kit.servo[11].angle=int(self.grip_angle)

    def rotate_servo(self, rotate):
        if(rotate==1):
            self.rotate_angle = self.rotate_angle+5
            if (self.rotate_angle >= 180):
                self.rotate_angle = 180
                kit.servo[14].angle=int(self.rotate_angle)
            else:
                kit.servo[14].angle=int(self.rotate_angle)
        if(rotate==(-1)):
            self.rotate_angle = self.rotate_angle-5
            if (self.rotate_angle <= 0):
                self.rotate_angle = 0
                kit.servo[14].angle=int(self.rotate_angle)
            else:
                kit.servo[14].angle=int(self.rotate_angle)       

    def ps4_deadband(self, ps4_value):
        if((ps4_value < 5) and (ps4_value > -5)):
            ps4_value = 0
        return ps4_value


    def joy_callback(self, msg:Joy):
        # Extract values from joystick
        base_pos = int(msg.buttons[2])
        base_neg = int(msg.buttons[0])

        mid_pos = int(msg.buttons[3])
        mid_neg = int(msg.buttons[1])

        upp_pos = int(msg.buttons[5])
        upp_neg = int(msg.buttons[4])


        grip = int(msg.axes[7])

        rotate = int(msg.axes[6])

        self.drive_base_servo(base_pos, base_neg)
        self.drive_mid_servo(mid_pos, mid_neg)
        self.drive_upp_servo(upp_pos, upp_neg)
        self.grip_servo(grip)
        self.rotate_servo(rotate)

        print(self.base_angle, self.mid_angle,
              self.upp_angle, self.grip_angle,
              self.rotate_angle)
        

def main():
    rclpy.init()
    servo_control = ServoControl()
    rclpy.spin(servo_control)
    servo_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
