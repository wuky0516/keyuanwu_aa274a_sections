#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Constant_control(Node):
    def __init__(self) -> None:
        super().__init__("constant_control")
        self.get_logger().info("Publisher has been created")
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Bool, '/kill', self.kill_callback, 10)
        
        self.timer = self.create_timer(0.2, self.callback)
        
        self.linear_velocity = 0.3
        self.angular_velocity = 0.2

    def callback(self) -> None:
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.pub.publish(msg)

    def kill_callback(self, msg: Bool) -> None:
        if msg.data:  # If the received message is True
            self.get_logger().info("Kill signal received! Stopping the robot.")
            
            # Stop the timer (stopping periodic publication)
            self.timer.cancel()
            
            # Publish zero velocities to stop the robot
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.pub.publish(stop_msg)  # Publish zero velocity to stop the robot

            self.get_logger().info("Robot has been stopped.")

if __name__ == "__main__":
    rclpy.init()
    node = Constant_control()
    rclpy.spin(node)
    rclpy.shutdown()

