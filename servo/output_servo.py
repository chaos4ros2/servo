import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
# from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int8

import Adafruit_PCA9685

ANGLE_MIN = -70
ANGLE_MAX = 70

# connect servo to PCA9685 (pin 0 and 3)
SERVO_LEFT_ID = 0
SERVO_RIGHT_ID = 3

class Servo(Node):
    def __init__(self, servo_name, servo_id):
        super().__init__('servo_node_' + servo_name)
        self.init_pca9685() 
        self.servo_name = servo_name
        self.servo_id = servo_id
        self.sub_topic_name = '/output/servo/' + servo_name
        self.sub_servo = self.create_subscription(Int8, self.sub_topic_name, self.servo_callback, 10)

    def init_pca9685(self):
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        self.pwm.set_pwm_freq(60)


    def servo_callback(self, servo_msg):
        self.get_logger().info('subscribe servo angle: {}'.format(servo_msg.data))
        self.set_angle(servo_msg.data)

    def set_angle(self, angle):
        angle = max(ANGLE_MIN, angle)
        angle = min(ANGLE_MAX, angle)
        pulse = (600-150) / 180 * (angle + 90) + 150
        self.pwm.set_pwm(self.servo_id, 0, int(pulse))


def main(args=None):
    rclpy.init(args=args)

    executor = SingleThreadedExecutor()
    ## use mutli thread
    # executor = MultiThreadedExecutor(num_threads=2)

    ## turn node (class) to instance and register to executor
    node_left = Servo(servo_name='left', servo_id=SERVO_LEFT_ID)
    node_right = Servo(servo_name='right', servo_id=SERVO_RIGHT_ID)
    executor.add_node(node_left)
    executor.add_node(node_right)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    node_left.destroy_node()
    node_right.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
