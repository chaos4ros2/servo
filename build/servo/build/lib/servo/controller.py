import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32

import random

ANGLE_MIN = -70
ANGLE_MAX = 70

class MyController(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.pub_servo_left = self.create_publisher(Float32, '/output/servo/left', 10)
        self.pub_servo_right = self.create_publisher(Float32, '/output/servo/right', 10)
        self.tmr = self.create_timer(1.0, self.move_servo)

    # for test(no sensor)
    def move_servo(self):
        self.pub_servo_left.publish(Float32(data=float(random.randrange(ANGLE_MIN, ANGLE_MAX))))
        self.pub_servo_right.publish(Float32(data=float(random.randrange(ANGLE_MIN, ANGLE_MAX))))

def main(args=None):
    rclpy.init(args=args)
    node = MyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()