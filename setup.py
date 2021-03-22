from glob import glob
from setuptools import setup

package_name = 'pigpio'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ## 後で作成する 'launch/*.launch.py'用に追記
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = pigpio.hello_world:main',
            'servo = pigpio.output_servo:main',
            'controller = pigpio.controller:main',
        ],
    },
)
