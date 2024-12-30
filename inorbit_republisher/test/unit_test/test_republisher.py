#!/usr/bin/env python3
# Unit test for a ROS node that republishes messages from /input_topic to /output_topic.
# It uses the unittest framework and rclpy for testing.

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestRepublisher(unittest.TestCase):
    def setUp(self):
        # Initialize the ROS 2 node for testing
        rclpy.init()
        self.node = rclpy.create_node('test_republisher')
        self.test_pub = self.node.create_publisher(String, '/input_topic', 10)
        self.received_messages = []
        self.node.create_subscription(String, '/output_topic', self.callback, 10)

        # Safe way to wait for publisher connections to be established
        timeout_seconds = 5.0
        end_time = self.node.get_clock().now() + rclpy.time.Duration(seconds=timeout_seconds)
        while self.test_pub.get_subscription_count() == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.get_clock().now() >= end_time:
                self.fail("Test setup failed: Publisher connection timeout.")

    def callback(self, msg):
        # Method to save messages received on /output_topic.
        self.received_messages.append(msg.data)

    def test_message_republishing(self):
        # Method to send a message (key=value) to /input_topic and
        # check if the republisher correctly adds the prefix (input_to_output=) and sends it to /output_topic.
        test_message = "key=value"
        expected_message = f"input_to_output={test_message}"
        self.node.get_logger().info(f"Publishing: {test_message} to /input_topic")
        self.test_pub.publish(String(data=test_message))

        # Wait for the message to be received
        timeout_seconds = 5.0
        end_time = self.node.get_clock().now() + rclpy.time.Duration(seconds=timeout_seconds)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if expected_message in self.received_messages:
                break

        self.node.get_logger().info(f"Messages received: {self.received_messages}")
        self.assertIn(expected_message, self.received_messages)

    def tearDown(self):
        # Clean up the ROS 2 node
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import ros2pytest 
    unittest.main()
