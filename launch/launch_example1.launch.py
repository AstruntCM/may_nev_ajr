from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='may_nev_ajr',
            executable='trafficLight',
            name='trafficLight',
            parameters=[{
                "greenLightTime": 3.0,
                "yellowLightTime": 2.0,
                "redLightTime": 3.0
            }]
        ),
        Node(
            package='may_nev_ajr',
            executable='driverNode',
            name='driverNode',
        )
    ])