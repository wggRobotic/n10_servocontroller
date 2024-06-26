from Adafruit_PCA9685 import PCA9685
from board import SCL, SDA
import math
import rclpy
import busio

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ServoController(Node):

    def __init__(self):

        super().__init__('n10_servo_controller')

        self.i2c_bus = busio.I2C(SCL,SDA)
        self.pca = PCA9685(self.i2c_bus)
        self.pca.frequency = 60

        self.wheel_servo_channels = [0, 1, 2, 3, 4, 5]
        self.arm_servo_channels = [6, 7, 8]

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
                    if 0 <= i < len(self.wheel_servo_channels):
                        duty_cycle = self.angle_to_duty_cycle(angle)
                        self.pca.channels[self.wheel_servo_channels[i]].duty_cycle = duty_cycle
                        self.get_logger().info(f'Set wheel servo {i} angle to {angle} degrees')


    def arm_control_callback(self, msg):
        if len(msg.data) == 3:
            for i, angle in enumerate(msg.data):
                if 0 <= i < len(self.arm_servo_channels):
                    duty_cycle = self.angle_to_duty_cycle(angle)
                    self.pca.channels[self.arm_servo_channels[i]].duty_cycle = duty_cycle
                    self.get_logger().info(f'Set arm servo {i} angle to {angle} degrees')


    def angle_to_duty_cycle(self, angle_rad):
        angle_deg = math.degrees(angle_rad)
        
        min_angle = -90.0
        max_angle = 90.0

        angle_deg = max(min(angle_deg, max_angle), min_angle)
     
        min_duty = 0x199A  # Corresponds to 5% of 0xFFFF
        max_duty = 0x3333  # Corresponds to 10% of 0xFFFF
    
        duty_cycle = int(((angle_deg - min_angle) / (max_angle - min_angle)) * (max_duty - min_duty) + min_duty)
        return duty_cycle

def main(args=None):
    rclpy.init(args=args)

    n10_servo_controller = ServoController()
    rclpy.spin(n10_servo_controller)

    n10_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
