import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from n10_servocontroller import PCAController

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
            
def main(args=None):
    rclpy.init(args=args)

    n10_servo_controller = N10ServoController()
    
    try:
        n10_servo_controller.run()
    except KeyboardInterrupt:
        pass 
    n10_servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
