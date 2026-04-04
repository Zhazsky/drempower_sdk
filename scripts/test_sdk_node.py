#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from drempower_sdk.msg import MotorCommand
import time

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('test_sdk_node')
        self.cmd_pub = self.create_publisher(MotorCommand, 'motor_commands', 10)
        self.state_sub = self.create_subscription(JointState, 'motor_states', self.state_callback, 10)
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.target_angle = 30.0
        self.get_logger().info("Test SDK Node started")

    def state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.get_logger().info(f"State: {name} - Angle: {msg.position[i]:.2f}, Speed: {msg.velocity[i]:.2f}, Torque: {msg.effort[i]:.2f}")

    def timer_callback(self):
        msg = MotorCommand()
        # Test Multi-Motor Step Position
        msg.motor_ids = [1] # Can add more IDs here if available
        msg.type = 4       # Step Position (Relative)
        msg.mode = 1       # Trapezoidal
        msg.values = [float(self.target_angle), 50.0, 40.0] # angle, speed, accel
        
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent Step command: {self.target_angle} degrees to motors {msg.motor_ids}")
        self.target_angle = -self.target_angle  # Toggle

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
