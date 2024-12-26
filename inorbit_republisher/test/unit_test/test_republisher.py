#!/usr/bin/env python
# Unit test for a ROS node that republishes messages from /input_topic to /output_topic. 
# It uses the unittest framework and rostest for testing.

import unittest
import rospy
from std_msgs.msg import String

class TestRepublisher(unittest.TestCase):
    def setUp(self):
        # Method to set up ROS nodes used in this test.
        rospy.init_node('test_republisher', anonymous=True)
        self.test_pub = rospy.Publisher('/input_topic', String, queue_size=10)
        self.received_messages = []
        rospy.Subscriber('/output_topic', String, self.callback)
        rospy.sleep(1)

    def callback(self, msg):
        # Method to save messages received on /output_topic.
        self.received_messages.append(msg.data)

    def test_message_republishing(self):
        # Method to send a message (key=value) to /input_topic and 
        # check if the republisher correctly adds the prefix (input_to_output=) and sends it to /output_topic.
        test_message = "key=value"
        expected_message = f"input_to_output={test_message}" 
        rospy.loginfo(f"Publishing: {test_message} to /input_topic")
        self.test_pub.publish(String(data=test_message))
        rospy.sleep(1)
        rospy.loginfo(f"Messages received: {self.received_messages}")
        self.assertIn(expected_message, self.received_messages)
if __name__ == '__main__':
    import rostest
    rostest.rosrun('my_ros_package', 'test_republisher', TestRepublisher)
