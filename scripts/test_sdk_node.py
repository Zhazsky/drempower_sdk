#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from drempower_sdk.msg import MotorCommand
import math

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('test_sdk_node')
        self.cmd_pub = self.create_publisher(MotorCommand, 'motor_commands', 10)
        self.state_sub = self.create_subscription(JointState, 'motor_states', self.state_callback, 10)
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.target_angle = 0.5236 # ~30 degrees in rad
        self.get_logger().info("Test SDK Node started (Radians mode)")

    def state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.get_logger().info(f"State: {name} - Pos: {msg.position[i]:.3f} rad, Vel: {msg.velocity[i]:.3f} rad/s, Torque: {msg.effort[i]:.3f} Nm")

    def timer_callback(self):
        msg = MotorCommand()
        # Test Multi-Motor Step Position
        msg.motor_ids = [1] # Can add more IDs here if available
        msg.type = 4       # Step Position (Relative)
        msg.mode = 1       # Trapezoidal
        msg.values = [float(self.target_angle), 2.0, 5.0] # [angle(rad), speed(rad/s), accel(rad/s^2)]
        
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent Step command: {self.target_angle:.3f} rad to motors {msg.motor_ids}")
        self.target_angle = -self.target_angle  # Toggle

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
