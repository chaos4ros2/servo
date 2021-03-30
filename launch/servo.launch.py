import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'urdf/servo.urdf.xml'

    print("urdf_file_name : {}".format(urdf_file_name))

    # shareパッケージに入れないため、パッケージ直下のパスを取得して使う
    # https://github.com/ament/ament_index/blob/master/ament_index_python/ament_index_python/packages.py
    urdf = os.path.join(get_package_prefix('servo'), urdf_file_name)

    return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{'use_sim_time': use_sim_time}],
          arguments=[urdf]),
        Node(
            package='servo',
            executable='servo',
            output='screen'
        ),
        Node(
            package='servo',
            executable='controller',
        )
    ])