from adafruit_servokit import ServoKit
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ServoController(Node):

    def __init__(self):

        super().__init__('n10_servo_controller')

        self.kit = ServoKit(channels=16)
        for i in range(9):
            self.kit.servo[i].set_pulse_width_range(550, 2500)

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
            '/n8/servo_cmd_arm',
            self.arm_control_callback,
            10
        )

        self.get_logger().info('n10_servo_controller started. listening ...')


    def wheel_angle_callback(self, msg):
        if len(msg.data) == 6:
            for i, angle in enumerate(msg.data):
                if 0 <= i < len(self.wheel_servo_channels):
                    move_to_angle(angle_servo_channels[i], angle)

    def arm_control_callback(self, msg):
        if len(msg.data) == 3:
            for i, angle in enumerate(msg.data):
                if 0 <= i < len(self.arm_servo_channels):
                    move_to_angle(arm_servo_channels[i], angle)

    def move_to_angle(channel, angle):
        self.kit.servo[channel].angle = angle


def main(args=None):
    rclpy.init(args=args)

    n10_servo_controller = ServoController()
    rclpy.spin(n10_servo_controller)

    n10_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
