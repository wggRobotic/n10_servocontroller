from adafruit_servokit import ServoKit
import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

DUTY_MID = 5200
DEG180_DUTY_RANGE = -3300
DEG270_DUTY_RANGE = 2000

class ServoController(Node):

    def __init__(self):

        super().__init__('n10_servo_controller')

        self.wheel_servo_channels = [1, 2, 6, 3, 5, 4]
        self.arm_servo_channels = [7, 8, 9]
        
        self.kit = ServoKit(channels=16)

        self.last_wheel_angles = []

        for i, channel in enumerate(self.wheel_servo_channels):
            self.kit.servo[channel]._pwm_out.duty_cycle = DUTY_MID
            self.last_wheel_angles[i] = DUTY_MID

        self.kit.servo[self.arm_servo_channels[0]]._pwm_out.duty_cycle = DUTY_MID
        self.kit.servo[self.arm_servo_channels[1]]._pwm_out.duty_cycle = DUTY_MID
        self.kit.servo[self.arm_servo_channels[2]]._pwm_out.duty_cycle = DUTY_MID


        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/n10/servo_cmd_wheels',
            self.wheel_angle_callback,
            10
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/n10/servo_cmd_arm',
            self.arm_control_callback,
            10
        )

        self.get_logger().info('n10_servo_controller started. listening ...')


    def wheel_angle_callback(self, msg):
        if len(msg.data) == 6:
            for i, angle in enumerate(msg.data):
                if -1.6 <= angle <= 1.6:
                    if(self.last_wheel_angles[i] != angle):
                        self.kit.servo[self.wheel_servo_channels[i]]._pwm_out.duty_cycle = int(DUTY_MID + angle / math.pi * 2 * DEG180_DUTY_RANGE)
                        self.last_wheel_angles[i] = angle

    def arm_control_callback(self, msg):
        if len(msg.data) == 3:
                if -2.3 <= msg.data[0] <= 2.3:
                    self.kit.servo[self.arm_servo_channels[0]]._pwm_out.duty_cycle = int(DUTY_MID - msg.data[0] / math.pi * 2 * DEG270_DUTY_RANGE)
                if -2.3 <= msg.data[1] <= 2.3:
                    self.kit.servo[self.wheel_servo_channels[1]]._pwm_out.duty_cycle = int(DUTY_MID + msg.data[1] / math.pi * 2 * DEG270_DUTY_RANGE)
                if -1.6 <= msg.data[2] <= 1.6:
                    self.kit.servo[self.wheel_servo_channels[2]]._pwm_out.duty_cycle = int(DUTY_MID + msg.data[2] / math.pi * 2 * DEG180_DUTY_RANGE)

def main(args=None):
    rclpy.init(args=args)

    n10_servo_controller = ServoController()
    
    try:
        rclpy.spin(n10_servo_controller)
    except KeyboardInterrupt:
        pass 
    n10_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
