#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class IRProcessorNode(Node):
    """
    Node that processes IR sensor data and sends it to the obstacle avoidance node.
    
    Subscribes to:
      - /ir_sensors (Int32): 3-digit integer representing 3 IR sensors
        (e.g., 101 means left and right sensors are active, center is inactive)
    
    Publishes to:
      - /ir_processed (Int32): Same data for simplicity, but could be processed differently if needed
    """
    
    def __init__(self):
        super().__init__('ir_processor_node')
        
        # Create subscriber to IR sensor topic
        self.ir_subscription = self.create_subscription(
            Int32,
            '/IR_vals',
            self.ir_callback,
            10
        )
        
        # Create publisher to send processed IR data
        self.ir_publisher = self.create_publisher(
            Int32,
            '/ir_processed',
            10
        )
        
        self.get_logger().info('IR Processor Node initialized')
        
    def ir_callback(self, msg):
        """Process incoming IR sensor data."""
        ir_value = msg.data
        
        # Log the received IR value
        self.get_logger().info(f'Received IR value: {ir_value}')
        
        # Decode the IR value
        left_sensor = (ir_value // 100) % 10  # Hundreds digit
        center_sensor = (ir_value // 10) % 10  # Tens digit
        right_sensor = ir_value % 10  # Units digit
        
        self.get_logger().info(f'Sensors: Left={left_sensor}, Center={center_sensor}, Right={right_sensor}')
        
        # For simplicity, we're just passing through the data
        # In a real application, you might do some filtering or preprocessing here
        processed_msg = Int32()
        processed_msg.data = ir_value
        
        # Publish the processed data
        self.ir_publisher.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = IRProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
