#!/usr/bin/env python

import unittest
import rospy
from std_msgs.msg import String

class TestRepublisher(unittest.TestCase):
    def setUp(self):
        #Inits a ROS node
        rospy.init_node('test_republisher', anonymous=True)
        # Sets a republisher and a subscriber node
        self.test_pub = rospy.Publisher('/input_topic', String, queue_size=10)
        self.received_messages = []
        # Subscribes to a specific topic
        rospy.Subscriber('/output_topic', String, self.callback)
        rospy.sleep(1)

    def callback(self, msg):
        # Callback method to caught message in  /output_topic
        self.received_messages.append(msg.data)

    def test_message_republishing(self):
        # Publish message in topic
        test_message = "key=value"
        self.test_pub.publish(String(data=test_message))
        rospy.sleep(1)
        # Verify message reception
        self.assertIn(test_message, self.received_messages)

    def test_no_message(self):
        # Verify no message was inicially recieved 
        self.assertEqual(len(self.received_messages), 0)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('my_ros_package', 'test_republisher', TestRepublisher)
