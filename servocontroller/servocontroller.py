from adafruit_servokit import ServoKit
import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

DUTY_MID = 5200
DUTY_RANGE = -3300

class ServoController(Node):

    def __init__(self):

        super().__init__('n10_servo_controller')
 
        self.wheel_servo_channels = [1, 2, 6, 3, 4, 5]
        self.arm_servo_channels = [6, 7, 8]

        self.kit = ServoKit(channels=16)

        self.target_dutys = []
        self.current_dutys = []

        for i in range(15):
            self.kit.servo[i]._pwm_out.duty_cycle = DUTY_MID
            self.target_dutys.append(DUTY_MID)
            self.current_dutys.append(DUTY_MID)
        
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

        self.timer = self.create_timer(0.01, self.servos_move_step)

        self.get_logger().info('n10_servo_controller started. listening ...')


    def wheel_angle_callback(self, msg):
        if len(msg.data) == 6:
            for i, angle in enumerate(msg.data):
                self.target_dutys[self.wheel_servo_channels[i]] = int(DUTY_MID + angle / math.pi * 2 * DUTY_RANGE)

    def arm_control_callback(self, msg):
        if len(msg.data) == 3:
            for i, angle in enumerate(msg.data):
                self.target_dutys[self.arm_servo_channels[i]] = int(DUTY_MID + angle / math.pi * 2 * DUTY_RANGE)

    def servos_move_step(self):
        servo_currently_moving_flag = False
        for i in range(15):
            diff = self.target_dutys[i] - self.current_dutys[i] 
            if diff != 0:
                self.servo_currently_moving_flag = True
                if diff < 100 or diff > 100:
                    if diff > 0:
                        self.current_dutys[i] += 50
                    else:
                        self.current_dutys[i] -= 50
                else:
                    self.current_dutys[i] = self.target_dutys[i]

                self.kit.servo[i]._pwm_out.duty_cycle = self.current_dutys[i]


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
