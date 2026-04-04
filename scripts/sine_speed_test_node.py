#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from drempower_sdk.msg import MotorCommand
import math
import time
import threading
import sys
import select
import termios
import tty

class SineSpeedTestNode(Node):
    def __init__(self):
        super().__init__('sine_speed_test_node')
        
        # 声明参数
        self.declare_parameter('motor_id', 1)
        self.motor_id = self.get_parameter('motor_id').get_parameter_value().integer_value
        
        # 发布与订阅
        self.cmd_pub = self.create_publisher(MotorCommand, 'motor_commands', 10)
        self.state_sub = self.create_subscription(JointState, 'motor_states', self.state_callback, 10)
        
        # 60Hz 定时器
        self.timer = self.create_timer(1.0/60.0, self.timer_callback)
        
        # 状态机变量
        self.state = 'INIT' 
        self.current_pos = 0.0 # rad
        self.start_sine_time = None
        self.running = True
        
        # 保存终端设置以便恢复
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 启动按键监听线程
        self.kb_thread = threading.Thread(target=self.keyboard_listener)
        self.kb_thread.daemon = True
        self.kb_thread.start()
        
        self.get_logger().info(f"正弦速度测试节点启动，电机ID: {self.motor_id}")
        self.get_logger().info("提示：按下 'q' 键或 'ESC' 键可安全停止电机并退出程序")

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        # 创建新的终端设置，禁用规范模式 (ICANON) 和回显 (ECHO)
        # 关键：保留 OPOST 输出处理，确保 \n 依然能正确触发 \r\n 换行
        new_settings = termios.tcgetattr(fd)
        new_settings[3] = new_settings[3] & ~(termios.ICANON | termios.ECHO)
        
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
            while rclpy.ok() and self.running:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    if key in ['q', 'Q', '\x1b']: # 'q' 或 ESC
                        self.get_logger().info(f"检测到按键 '{key}'，正在停止电机并退出...")
                        self.stop_motor()
                        self.running = False
                        break
        finally:
            # 无论如何都要恢复原始终端设置
            termios.tcsetattr(fd, termios.TCSADRAIN, self.settings)

    def stop_motor(self):
        # 构建并发布速度为 0 的指令
        stop_msg = MotorCommand()
        stop_msg.motor_ids = [self.motor_id]
        stop_msg.type = 1
        stop_msg.mode = 0
        stop_msg.values = [0.0, 0.0]
        self.cmd_pub.publish(stop_msg)
        self.get_logger().info("已发送停止指令")

    def state_callback(self, msg):
        motor_name = f"motor_{self.motor_id}"
        if motor_name in msg.name:
            idx = msg.name.index(motor_name)
            self.current_pos = msg.position[idx]

    def timer_callback(self):
        if not self.running:
            # 如果已触发退出，停止定时器逻辑
            return

        if self.state == 'INIT':
            # 1. 要求电机回到零点
            msg = MotorCommand()
            msg.motor_ids = [self.motor_id]
            msg.type = 0  # 绝对角度控制
            msg.mode = 1  # 梯形轨迹模式
            msg.values = [0.0, 2.0, 5.0] # [目标角度(rad), 目标速度(rad/s), 加速度(rad/s^2)]
            self.cmd_pub.publish(msg)
            self.get_logger().info("正在指令电机回归零点...")
            self.state = 'MOVING_TO_ZERO'

        elif self.state == 'MOVING_TO_ZERO':
            # 2. 确认到零点后再发布速度 (容差 0.01 弧度 ≈ 0.57度)
            if abs(self.current_pos) < 0.01:
                self.get_logger().info("电机已到达零点，开始正弦速度测试")
                self.state = 'SINE_WAVE'
                self.start_sine_time = time.time()
            else:
                # 持续发送归零指令防止丢帧（可选）
                pass

        elif self.state == 'SINE_WAVE':
            # 3. 连续 60Hz 发布速度 (正弦波)
            t = time.time() - self.start_sine_time
            T = 6
            # v = A * sin(2 * pi * f * t), 其中 f = 1/T
            speed = 2.0 * math.sin(2.0 * math.pi * (1.0/T) * t) # Amplitude 2.0 rad/s
            
            msg = MotorCommand()
            msg.motor_ids = [self.motor_id]
            msg.type = 1  # 速度控制
            msg.mode = 0  # 直接/前馈模式
            msg.values = [float(speed), 0.0] # [目标速度(rad/s), 前馈力矩]
            
            self.cmd_pub.publish(msg)
            # 为了减少日志量，可以不每帧打印
            if int(t * 60) % 60 == 0: # 约每秒打印一次
                self.get_logger().info(f"正弦速度指令: {speed:.3f} rad/s, 当前角度: {self.current_pos:.3f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = SineSpeedTestNode()
    
    # 使用循环检查 running 状态
    while rclpy.ok() and node.running:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # 退出前的最终清理
    # 恢复终端设置
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
