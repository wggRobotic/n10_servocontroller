from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import math

class PCAController:
    def __init__(self, node):

        self.node = node

        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50

        node.declare_parameter('wheel_channel_mappings', [0, 1, 2, 3, 4, 5])
        node.declare_parameter('wheel_duty_mids', [ 5200,  5200,  4900,  5000,  4800,  4800])
        node.declare_parameter('wheel_duty_ranges_positive', [-3600, -3500, -3400, -3400, -3400, -3500])
        node.declare_parameter('wheel_duty_ranges_negative', [-3100, -3300, -3200, -3400, -3400, -3400])

        node.declare_parameter('gripper_channel_mappings', [0, 1, 2, 3])
        node.declare_parameter('gripper_duty_mids', [ 5200,  5200,  4900,  5000])
        node.declare_parameter('gripper_duty_ranges_positive', [0, 0, 0, 0])
        node.declare_parameter('gripper_duty_ranges_negative', [0, 0, 0, 0])

        self.wheel_channel_mappings =       node.get_parameter('wheel_channel_mappings').get_parameter_value().integer_array_value
        self.wheel_duty_mids =              node.get_parameter('wheel_duty_mids').get_parameter_value().integer_array_value
        self.wheel_duty_ranges_positive =   node.get_parameter('wheel_duty_ranges_positive').get_parameter_value().integer_array_value
        self.wheel_duty_ranges_negative =   node.get_parameter('wheel_duty_ranges_negative').get_parameter_value().integer_array_value

        self.gripper_channel_mappings =     node.get_parameter('gripper_channel_mappings').get_parameter_value().integer_array_value
        self.gripper_duty_mids =            node.get_parameter('gripper_duty_mids').get_parameter_value().integer_array_value
        self.gripper_duty_ranges_positive = node.get_parameter('gripper_duty_ranges_positive').get_parameter_value().integer_array_value
        self.gripper_duty_ranges_negative = node.get_parameter('gripper_duty_ranges_negative').get_parameter_value().integer_array_value

        self.wheel_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper_angles = [0.0, 0.0, 0.0, 0.0]

        #self.set_wheel_angles(self.wheel_angles)
        #self.set_gripper_angles(self.gripper_angles)

    def set_wheel_angles(self, angles):
        
        for i in range(6):

            duty_cycle = self.wheel_duty_mids[i]

            if angles[i] > 0:
                duty_cycle += int(self.wheel_duty_ranges_positive[i] * angles[i] / math.pi * 2)
            else:
                duty_cycle += int(self.wheel_duty_ranges_negative[i] * angles[i] / math.pi * 2)

            self.pca.channels[self.wheel_channel_mappings[i]].duty_cycle = duty_cycle
            self.wheel_angles[i] = angles[i]

    def get_wheel_angles(self):
        return self.wheel_angles
    
    def set_gripper_angles(self, angles):
        
        for i in range(4):

            duty_cycle = self.gripper_duty_mids[i]

            if angles[i] > 0:
                duty_cycle += int(self.gripper_duty_ranges_positive[i] * angles[i] / math.pi * 2)
            else:
                duty_cycle += int(self.gripper_duty_ranges_negative[i] * angles[i] / math.pi * 2)

            self.pca.channels[self.gripper_channel_mappings[i]].duty_cycle = duty_cycle
            self.gripper_angles[i] = angles[i]

    def get_gripper_angles(self):
        return self.gripper_angles

    def disable_wheel_servos(self):
        for i in range(6):
            self.pca.channels[self.wheel_channel_mappings[i]].duty_cycle = 0

    def disable_gripper_servos(self):
        for i in range(4):
            self.pca.channels[self.gripper_channel_mappings[i]].duty_cycle = 0


#from n10_servocontroller import PCAController

class N10ServoController(Node):

    def __init__(self):

        super().__init__('n10_servocontroller')

    def run(self):

        self.declare_parameter('mode', 'none')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        if self.mode == 'pca':
            self.controller = PCAController(self)
        else:
            self.get_logger().info('couldn\'t read servo mode. Aborting ...')
            return

        self.wheels_subscription = self.create_subscription(
            Float32MultiArray,
            'wheels/angle/cmd',
            self.wheels_angle_callback,
            10
        )

        self.gripper_subscribtion = self.create_subscription(
            Float32MultiArray,
            'gripper/angle/cmd',
            self.gripper_angle_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.feedback_timer_callback)

        self.wheels_feedback_publisher = self.create_publisher(
            Float32MultiArray,
            'wheels/angle/feedback',
            10,
        )

        self.gripper_feedback_publisher = self.create_publisher(
            Float32MultiArray,
            'gripper/angle/feedback',
            10,
        )

        self.get_logger().info('n10_servocontroller started. listening ...')

        rclpy.spin(self)


    def wheels_angle_callback(self, msg):
        if len(msg.data) == 6:
            self.controller.set_wheel_angles(msg.data)

    def gripper_angle_callback(self, msg):
        if len(msg.data) == 4:
            self.controller.set_gripper_angles(msg.data)

    def feedback_timer_callback(self):

        wheels_msg = Float32MultiArray()
        wheels_msg.data = self.controller.get_wheel_angles()
        self.wheels_feedback_publisher.publish(wheels_msg)

        gripper_msg = Float32MultiArray()
        gripper_msg.data = self.controller.get_gripper_angles()
        self.gripper_feedback_publisher.publish(gripper_msg)
        
    def disable_servos(self):
        self.controller.disable_wheel_servos()
        self.controller.disable_gripper_servos()



def main(args=None):
    rclpy.init(args=args)

    n10_servo_controller = N10ServoController()
    
    try:
        n10_servo_controller.run()
    except KeyboardInterrupt:
        pass 
    n10_servo_controller.disable_servos()
    n10_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
