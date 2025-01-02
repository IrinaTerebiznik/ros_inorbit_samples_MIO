#!/usr/bin/env python3
import os
import time
import unittest

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
from ament_index_python.packages import get_package_share_directory
@pytest.mark.launch_test
def generate_test_description():
    # Absolute path to the launch file
    launch_file_path = '/root/ros2_ws/install/inorbit_republisher/share/inorbit_republisher/test_republisher.launch.xml'

    # Include the launch file in the test description
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file_path)
        ),
        launch_testing.actions.ReadyToTest(),
    ])
class TestRepublisher(unittest.TestCase):
    """A normal unittest, but under the umbrella of launch_testing."""

    def setUp(self):
        # Here the republisher node *should* already be running, thanks to generate_test_description()
        rclpy.init()
        self.node = rclpy.create_node('test_republisher')
        self.test_pub = self.node.create_publisher(String, '/input_topic', 10)
        self.received_messages = []

        self.node.create_subscription(String, '/output_topic', self.callback, 10)

        # Wait up to 5 seconds for at least one subscriber on /input_topic
        end_time = time.time() + 5
        while time.time() < end_time:
            if self.test_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)
        else:
            self.fail("Test setup failed: Publisher connection timeout.")

    def callback(self, msg):
        self.received_messages.append(msg.data)

    def test_message_republishing(self):
        test_message = "key=value"
        expected_message = f"input_to_output={test_message}"
        self.node.get_logger().info(f"Publishing: {test_message} to /input_topic")

        self.test_pub.publish(String(data=test_message))

        # Spin for up to 5 seconds waiting for the republished message
        end_time = time.time() + 5
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if expected_message in self.received_messages:
                break

        self.node.get_logger().info(f"Messages received: {self.received_messages}")
        self.assertIn(expected_message, self.received_messages)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Typically, colcon test or ros2 test will discover and run this via pytest
    # but you can still run it as a bare unittest if needed
    unittest.main()
