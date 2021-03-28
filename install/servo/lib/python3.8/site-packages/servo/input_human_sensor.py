import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import pigpio
import time

SENSOR_GPIO_PIN = 17 # 17番PINにセンサ接続
TIMER_INTERVAL = 0.5

class HumanSensor(Node):
    def __init__(self):
        super().__init__('human_sensor_node')
        self.init_sensor() # センサを使わない場合、不要
        self.pub_sensor = self.create_publisher(Bool, '/input/human_sensor', 10)
        self.timer = self.create_timer(TIMER_INTERVAL, self.sensor_timer_callback)
        self.prev_sensor_data = False


    def sensor_timer_callback(self):
        sensor_msg = Bool(data=self.get_sensor_data())
        self.pub_sensor.publish(sensor_msg)


    def init_sensor(self):
        self.pi = pigpio.pi()
        self.pi.set_mode(SENSOR_GPIO_PIN, pigpio.INPUT)
        self.pi.set_pull_up_down(SENSOR_GPIO_PIN, pigpio.PUD_UP)

    # センサを使わない場合、True/Falseを返す適当な関数に置きかえてください
    def get_sensor_data(self):
        if self.pi.read(SENSOR_GPIO_PIN) == 1:
            return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = HumanSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
