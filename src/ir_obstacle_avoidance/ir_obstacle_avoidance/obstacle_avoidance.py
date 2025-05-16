#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    """
    Node that implements basic obstacle avoidance logic based on IR sensor data.
    
    Subscribes to:
      - /ir_processed (Int32): 3-digit integer representing 3 IR sensors
    
    Publishes to:
      - /cmd_vel (Twist): Velocity commands for the robot
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Create subscriber to processed IR data
        self.ir_subscription = self.create_subscription(
            Int32,
            '/ir_processed',
            self.ir_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Default movement speeds
        self.linear_speed = 0.4  # m/s
        self.angular_speed = 2.5  # rad/s
        
        self.get_logger().info('Obstacle Avoidance Node initialized')
        
    def ir_callback(self, msg):
        """Process IR data and implement obstacle avoidance logic."""
        ir_value = msg.data
        
        # Decode the IR value
        left_sensor = (ir_value // 100) % 10    # Hundreds digit (1 = obstacle detected)
        center_sensor = (ir_value // 10) % 10   # Tens digit
        right_sensor = ir_value % 10            # Units digit
        
        # Create a Twist message
        twist_msg = Twist()
        
        # Implement basic obstacle avoidance logic
        # A value of 1 means an obstacle is detected
        
        # Case 1: No obstacles detected (000) - Move forward
        if left_sensor == 0 and center_sensor == 0 and right_sensor == 0:
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = 0.0
            self.get_logger().info('No obstacles - Moving forward')
            
        # Case 2: Center obstacle (010) - Stop and rotate
        elif left_sensor == 0 and center_sensor == 1 and right_sensor == 0:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_speed  # Turn left by default
            self.get_logger().info('Center obstacle - Rotating left')
            
        # Case 3: Left obstacle (100) - Turn right
        elif left_sensor == 1 and center_sensor == 0 and right_sensor == 0:
            twist_msg.linear.x = self.linear_speed * 0.5  # Slow down
            twist_msg.angular.z = -self.angular_speed     # Turn right
            self.get_logger().info('Left obstacle - Turning right')
            
        # Case 4: Right obstacle (001) - Turn left
        elif left_sensor == 0 and center_sensor == 0 and right_sensor == 1:
            twist_msg.linear.x = self.linear_speed * 0.5  # Slow down
            twist_msg.angular.z = self.angular_speed      # Turn left
            self.get_logger().info('Right obstacle - Turning left')
            
        # Case 5: Left and center obstacles (110) - Turn right sharply
        elif left_sensor == 1 and center_sensor == 1 and right_sensor == 0:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -self.angular_speed * 1.5  # Turn right sharply
            self.get_logger().info('Left and center obstacles - Turning right sharply')
            
        # Case 6: Right and center obstacles (011) - Turn left sharply
        elif left_sensor == 0 and center_sensor == 1 and right_sensor == 1:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_speed * 1.5  # Turn left sharply
            self.get_logger().info('Right and center obstacles - Turning left sharply')
            
        # Case 7: Left and right obstacles (101) - Move forward slowly if center is clear
        elif left_sensor == 1 and center_sensor == 0 and right_sensor == 1:
            twist_msg.linear.x = self.linear_speed * 0.3  # Move very slowly
            twist_msg.angular.z = 0.0
            self.get_logger().info('Left and right obstacles - Moving forward slowly')
            
        # Case 8: All obstacles (111) - Stop and rotate in place to find a way out
        else:  # This covers the 111 case
            twist_msg.linear.x = -0.1  # Move slightly backward
            twist_msg.angular.z = self.angular_speed * 2.0  # Rotate more aggressively
            self.get_logger().info('Surrounded by obstacles - Backing up and rotating')
            
        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ensure the robot stops when the node is terminated
        twist_msg = Twist()
        node.cmd_vel_publisher.publish(twist_msg)
        node.get_logger().info('Node stopped. Robot stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
