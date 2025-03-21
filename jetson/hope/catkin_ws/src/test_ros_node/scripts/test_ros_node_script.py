#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from test_ros_node.msg import blah

class TestNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('test_node', anonymous=True)
        
        # Publisher to /test_pub
        self.pub = rospy.Publisher('/test_pub', blah, queue_size=10)
        
        # Subscriber to /test_sub
        rospy.Subscriber('/test_sub', String, self.callback)
        
        # Publish an initial message
        
    def callback(self, msg):
        # Log the received message
        send_msg = blah()

        send_msg.first_name = "John"
        send_msg.last_name = "Doe"
        send_msg.age = 30
        send_msg.score = 95.5

        rospy.loginfo(f"Received message: {msg.data}")
        
        # Publish "received" to /test_pub
        self.pub.publish(send_msg)

if __name__ == '__main__':
    try:
        # Create an instance of the TestNode class
        node = TestNode()
        
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass