import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drempower_sdk',
            executable='drempower_node',
            name='drempower_node',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200,
                'motor_ids': [1],
                'update_rate': 20.0,
                'init_motor_settings': False,
                # 'p_gain': 30.0,
                # 'i_gain': 25.0,
                # 'd_gain': 20.0,
                # 'set_angle_range': True,
                # 'min_angle': -90.0,
                # 'max_angle': 90.0,
            }]
        ),
        # Node(
        #     package='drempower_sdk',
        #     executable='test_sdk_node.py',
        #     name='test_sdk_node',
        #     output='screen'
        # )
    ])
