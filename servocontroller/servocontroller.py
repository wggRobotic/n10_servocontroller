from adafruit_servokit import ServoKit
import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

DUTY_MID = 5200
DUTY_RANGE = 3300

kit = ServoKit(channels=16)

target_wheel_dutys = [DUTY_MID, DUTY_MID, DUTY_MID, DUTY_MID, DUTY_MID, DUTY_MID]
current_wheel_dutys = [DUTY_MID, DUTY_MID, DUTY_MID, DUTY_MID, DUTY_MID, DUTY_MID]

class ServoController(Node):

    def __init__(self):

        super().__init__('n10_servo_controller')

        for i in range(15):
            kit.servo[i]._pwm_out.duty_cycle = DUTY_MID
        
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

        self.timer = self.create_timer(0.02, self.wheel_servo_move)

        self.get_logger().info('n10_servo_controller started. listening ...')


    def wheel_angle_callback(self, msg):
        if len(msg.data) == 6:
            for i, angle in enumerate(msg.data):
                if 0 <= i < len(self.wheel_servo_channels):
                    target_wheel_dutys[i] = int(DUTY_MID + angle / math.pi * 2 * DUTY_RANGE)

    def arm_control_callback(self, msg):
        if len(msg.data) == 3:
            for i, angle in enumerate(msg.data):
                if 0 <= i < len(self.arm_servo_channels):
                    pass

    def wheel_servo_move(self):
        for i, channel in enumerate(self.wheel_servo_channels):
            diff = target_wheel_dutys[i] - current_wheel_dutys[i] 
            
            if diff != 0:
                if diff < 100 or diff > 100:
                    if diff > 0:
                        current_wheel_dutys[i] += 50
                    else:
                        current_wheel_dutys[i] -= 50
                else:
                    current_wheel_dutys[i] = target_wheel_dutys[i]

                kit.servo[channel]._pwm_out.duty_cycle = current_wheel_dutys[i]
            
def main(args=None):
    rclpy.init(args=args)

    n10_servo_controller = ServoController()
    
    try:
        rclpy.spin(n10_servo_controller)
    except KeyboardInterrupt:
        for i in range(15):
           kit.servo[i]._pwm_out.duty_cycle = DUTY_MID 
    
    n10_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
