"""
Launch file for ros node!
You don't technically need this but it is an easy way to get the node up and running super quickly and be able to set the parameters the way you want them!

Run instructions:
    ros2 launch pavbot_estop estop.launch.py

Check your nodes are working and publishing:
    (in another terminal)
    ros2 topic list
    ros2 topic echo safety/estop

    *should see a bunch of data:true*
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pavbot_imu',
            executable='pavbot_imu',
            name='pavbot_imu',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                'timeout_ms': 500
            }]
        )
    ])
