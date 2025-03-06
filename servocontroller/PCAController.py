import math
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

class PCAController:
    def __init__(self, node):

        self.node = node

        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50

        node.declare_parameter('wheel_channels', [0, 1, 2, 3, 4, 5])
        node.declare_parameter('wheel_duty_mids', [ 5200,  5200,  4900,  5000,  4800,  4800])
        node.declare_parameter('wheel_duty_ranges_positive', [-3600, -3500, -3400, -3400, -3400, -3500])
        node.declare_parameter('wheel_duty_ranges_negative', [-3100, -3300, -3200, -3400, -3400, -3400])

        self.wheel_channels = node.get_parameter('wheel_channels').get_parameter_value().int_array
        self.wheel_duty_mids = node.get_parameter('wheel_duty_mids').get_parameter_value().int_array
        self.wheel_duty_ranges_positive = node.get_parameter('wheel_duty_ranges_positive').get_parameter_value().int_array
        self.wheel_duty_ranges_negative = node.get_parameter('wheel_duty_ranges_negative').get_parameter_value().int_array

    def set_wheel_angles(self, angles):
        
        for i in range(6):

            angle = self.wheel_duty_mids[i]

            if angles[i] > 0:
                angle += int(self.wheel_duty_ranges_positive[i] * angles[i] / math.pi)
            else:
                angle += int(self.wheel_duty_ranges_negative[i] * angles[i] / math.pi)

            self.pca.channels[self.wheel_channels[i]].duty_cycle = angle