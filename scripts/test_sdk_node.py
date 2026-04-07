#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from drempower_sdk.msg import MotorCommand
import math
import threading
import sys

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('test_sdk_node')
        self.cmd_pub = self.create_publisher(MotorCommand, 'motor_commands', 10)
        self.state_sub = self.create_subscription(JointState, 'motor_states', self.state_callback, 10)
        
        self.get_logger().info("Test SDK Node 交互模式已启动")
        self.get_logger().info("请输入目标位置 (单位: 弧度 rad)，按 Enter 发送。输入 'q' 退出。")

        # 启动输入线程，避免阻塞 rclpy.spin()
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def state_callback(self, msg):
        # 仅在有状态更新时打印一次，避免刷屏
        pass

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("\n[请输入目标位置 (rad)]: ").strip()
                
                if user_input.lower() == 'q':
                    self.get_logger().info("退出程序...")
                    rclpy.shutdown()
                    break

                target_angle = float(user_input)
                self.send_command(target_angle)
                
            except ValueError:
                print("错误: 请输入有效的数字！")
            except EOFError:
                break

    def send_command(self, angle):
        msg = MotorCommand()
        msg.motor_ids = [1] 
        msg.type = 0      
        msg.mode = 1       # Trapezoidal
        msg.values = [float(angle), 1.0, 3.0] # [位置(rad), 速度(rad/s), 加速度(rad/s^2)]
        
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"已发送目标位置指令: {angle:.3f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
