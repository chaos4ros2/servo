import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
# from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int8, Float32

import Adafruit_PCA9685

# 3/28 plot servo to rviz2 with tf2
import tf2_ros
import geometry_msgs.msg
from rclpy.qos import QoSProfile
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion

# 3/30 add joint_state
from sensor_msgs.msg import JointState

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
        
        if 'left' in servo_name:
            self.joint_name = 'joint0'
        else:
            self.joint_name = 'joint1'

        self.servo_id = servo_id
        self.sub_topic_name = '/output/servo/' + servo_name

        self.qos_profile = QoSProfile(depth=10)
        self.sub_servo = self.create_subscription(Float32, self.sub_topic_name, self.servo_callback, 10)

        # joint_stateのpublisherを追加する
        self.joint_pub = self.create_publisher(JointState, 'joint_states', self.qos_profile)

    def init_pca9685(self):
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        self.pwm.set_pwm_freq(60)


    def servo_callback(self, servo_msg):
        self.get_logger().info('subscribe servo angle: {}'.format(servo_msg.data))
        # self.set_angle(servo_msg.data) selfに代入してるから、initに移行しても大丈夫そう
        self.set_angle(servo_msg.data)
        self.br = tf2_ros.TransformBroadcaster(self, qos = self.qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        t = geometry_msgs.msg.TransformStamped()
        now = self.get_clock().now()
        t.header.stamp = now.to_msg()

        # header.frame_idに土台になるid、childは一番最初のリンク名（土台）にする
        # t.header.frame_id = "base_link"
        # t.child_frame_id = self.joint_name
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # JointState()を追加する
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        # すべて渡さなくても大丈夫と書いてあるが、必要な部分の選出方法は分からない。
        # http://wiki.ros.org/ja/robot_state_publisher/Tutorials/Using%20the%20robot%20state%20publisher%20on%20your%20own%20robot
        # joint_state.positionに変数を代入する場合
        angle = 0.0
        angle = to_radians(servo_msg)

        joint_state.name = [self.joint_name]
        joint_state.position = [angle]

        # set posture(servo_msg is angle(Float32))
        # left is (0, 0, angle)
        # right is (0, angle, 0)
        # base_linkの位置を指定する(0, 0, 0)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0    
        t.transform.rotation = euler_to_quaternion(0, 0, 0) # roll,pitch,yaw
        self.joint_pub.publish(joint_state)
        self.br.sendTransform(t)
        
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


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def to_radians(deg_angle):
    return deg_angle.data * pi / 180.0

if __name__ == '__main__':
    main()
