from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

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

        self.set_wheel_angles(self.wheel_angles)
        self.set_gripper_angles(self.gripper_angles)

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