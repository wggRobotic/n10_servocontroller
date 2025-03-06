import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import PCAController

class ServoController(Node):

    def __init__(self):

        super().__init__('servocontroller')

    def run(self):

        self.declare_parameter('mode', 'none')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        if self.mode == 'pca':
            self.controller = PCAController(self)
        else:
            self.get_logger().info('couldn\'t read servo mode. Aborting ...')

        self.wheels_subscription = self.create_subscription(
            Float32MultiArray,
            'wheels/angle/cmd',
            self.wheels_angle_callback,
            10
        )

        self.gripper_subscribtion = self.create_subscription(
            Float32MultiArray,
            'gripper/cmd',
            self.arm_control_callback,
            10
        )

        self.wheels_feedback_publisher = self.create_publisher(
            Float32MultiArray,
            'wheels/angle/feedback',
        )

        self.gripper_feedback_publisher = self.create_publisher(
            Float32MultiArray,
            'wheels/angle/feedback',
        )

        self.get_logger().info('servocontroller started. listening ...')

        rclpy.spin(self)


    def wheels_angle_callback(self, msg):
        if len(msg.data) == 6:
            set_wheel_angles
            
def main(args=None):
    rclpy.init(args=args)

    n10_servo_controller = ServoController()
    
    try:
        n10_servo_controller.run()
    except KeyboardInterrupt:
        pass 
    n10_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
