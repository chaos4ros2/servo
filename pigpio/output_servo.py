import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
# from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int8

import Adafruit_PCA9685

ANGLE_MIN = -70
ANGLE_MAX = 70

# PCA9685 の 0 と 3 に、2つのサーボを接続
SERVO_LEFT_ID = 0
SERVO_RIGHT_ID = 3

class Servo(Node):
    def __init__(self, servo_name, servo_id):
        super().__init__('servo_node_' + servo_name)
        self.init_pca9685() # サーボを使わない場合、不要
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

    # サーボを使わない場合、適当な関数に置きかえてください
    def set_angle(self, angle):
        angle = max(ANGLE_MIN, angle)
        angle = min(ANGLE_MAX, angle)
        pulse = (600-150) / 180 * (angle + 90) + 150
        self.pwm.set_pwm(self.servo_id, 0, int(pulse))


def main(args=None):
    rclpy.init(args=args)

    executor = SingleThreadedExecutor()
    ## mutli thread の場合はこっち
    # executor = MultiThreadedExecutor(num_threads=2)

    ## ノード（クラス）を2つインスタンス化し、executor に登録
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
