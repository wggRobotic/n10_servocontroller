from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import numpy
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
        self.servo_channels = [0, 1, 2, 3, 4, 5]

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/n10/servo_cmd_wheels',
            self.wheel_angle_callback,
            10
        )
        self.subscription

        self.get_logger().info('n10_servo_controller started. listening ...')


    def wheel_angle_callback(self, msg):
        if len(msg.data) == 6:
            for i, angle in enumerate(msg.data):
                if 0 <= i < len(self.servo_channels):
                    duty_cycle = self.angle_to_duty_cycle(angle)
                    self.set_pwm(self.servo_channels[i], duty_cycle)
                    self.get_logger().info(f'Set servo {i} angle to {angle} degrees')

    def angle_to_duty_cycle(self, angle_rad):
        # Convert angle from radians (-pi/2 to pi/2) to duty cycle (0-65535)
        # Map [-pi/2, pi/2] radians to [0, 180] degrees
        angle_deg = math.degrees(angle_rad)
        # Convert [0, 180] degrees to [0, 65535] duty cycle
        duty_cycle = int((angle_deg / 180.0) * 65535.0)
        return duty_cycle

def main(args=None):
    rclpy.init(args=args)

    n10_servo_controller = ServoController()

    rclpy.spin(n10_servo_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    n10_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
