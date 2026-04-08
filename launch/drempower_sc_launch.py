import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明可配置参数
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN 接口名称 (例如 can0, vcan0)'
    )
    
    motor_ids_arg = DeclareLaunchArgument(
        'motor_ids',
        default_value='[1]',
        description='管理的电机 ID 列表 (例如 [1, 2, 3])'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='30.0',
        description='电机状态发布频率 (Hz)'
    )

    return LaunchDescription([
        can_interface_arg,
        motor_ids_arg,
        update_rate_arg,
        
        Node(
            package='drempower_sdk',
            executable='drempower_sc_node',
            name='drempower_sc_node',
            output='screen',
            parameters=[{
                'can_interface': LaunchConfiguration('can_interface'),
                'motor_ids': [1], # 注意：ROS2 Launch 传递整数数组有时存在限制，默认先写死或通过参数服务器
                'update_rate': LaunchConfiguration('update_rate'),
                'init_motor_settings': False,
                # 初始化配置示例 (取消注释以启用)
                # 'set_zero_position_temp': False,
                # 'min_angle': -3.14,
                # 'max_angle': 3.14,
                # 'set_angle_range': False,
                # 'speed_limit': -1.0,
                # 'torque_limit': -1.0,
            }],
            # 如果需要动态覆盖 motor_ids，建议在命令行或单独的 yaml 中指定
        ),
    ])
