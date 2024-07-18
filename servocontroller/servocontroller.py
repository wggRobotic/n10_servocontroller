from adafruit_servokit import ServoKit
import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ServoController(Node):

    def __init__(self):

        super().__init__('n10_servo_controller')

        self.kit = ServoKit(channels=16)

        self.wheel_servo_channels = [1, 2, 6, 3, 5, 4]
        self.wheel_angle_min = [-1.6, -1.6, -1.6, -1.6, -1.6, -1.6]
        self.wheel_angle_max = [1.6, 1.6, 1.6, 1.6, 1.6, 1.6]
        self.wheel_duty_mids = [5350, 5250, 5150, 5150, 5200, 5050]
        self.wheel_duty_ranges = [-3400, -3400, -3400, -3300, -3300, -3300]


        self.arm_servo_channels = [8, 9, 10, 11]
        self.arm_angle_min = [-2.3562, -2.3562, -1.6, -1.6]
        self.arm_angle_max = [0.27, 2.3562, 1.31, 1.6]
        self.arm_duty_mids = [5050, 5150, 5300, 5100]
        self.arm_duty_ranges = [2250, -2300, 3400, 3300]
        

        self.last_wheel_angles = []
        self.last_arm_angles = []
        
        for i in range(len(self.wheel_servo_channels)):
            self.last_wheel_angles.append(0)

        self.last_arm_angles.append(0.1326)
        self.last_arm_angles.append(-1.873)
        self.last_arm_angles.append(-1.33)
        self.last_arm_angles.append(0)


        for i, channel in enumerate(self.wheel_servo_channels):
            self.kit.servo[channel]._pwm_out.duty_cycle = int(self.wheel_duty_mids[i] + self.last_wheel_angles[i] / math.pi * 2 * self.wheel_duty_ranges[i])


        for i, channel in enumerate(self.arm_servo_channels):
            self.kit.servo[channel]._pwm_out.duty_cycle = int(self.arm_duty_mids[i] + self.last_arm_angles[i] / math.pi * 2 * self.arm_duty_ranges[i])


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
                if self.wheel_angle_min[i] <= angle <= self.wheel_angle_max[i]:
                    if(self.last_wheel_angles[i] != angle):
                        self.kit.servo[self.wheel_servo_channels[i]]._pwm_out.duty_cycle = int(self.wheel_duty_mids[i] + angle / math.pi * 2 * self.wheel_duty_ranges[i])
                        self.last_wheel_angles[i] = angle

    def arm_control_callback(self, msg):
        if len(msg.data) == 4:
            for i, angle in enumerate(msg.data):
                if self.arm_angle_min[i] <= angle <= self.arm_angle_max[i]:
                    if(self.last_arm_angles[i] != angle):
                        self.kit.servo[self.arm_servo_channels[i]]._pwm_out.duty_cycle = int(self.arm_duty_mids[i] + angle / math.pi * 2 * self.arm_duty_ranges[i])
                        self.last_arm_angles[i] = angle

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
