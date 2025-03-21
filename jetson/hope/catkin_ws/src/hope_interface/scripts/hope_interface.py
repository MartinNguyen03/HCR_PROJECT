#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

"""
======================================================================
Configuration Table
======================================================================
"""

CONFIG = {
    "text_to_speech_interface":
    {
        "subscriber_topics":
        {
            'text_input': '/hope/speak'
        },
        "publisher_topics":
        {
            'pepper_tts_topic': '/speech'
        },
    },

    "sensor_interface":
    {
        "subscriber_topics":
        {
            'foo': 'bar'
        },
        "publisher_topics":
        {
            'foo': 'bar'
        }
    },
}


"""
======================================================================
TextToSpeechInterface Class
======================================================================
"""

class TextToSpeechInterface(object):
    def __init__(self, config):
        # Retrieve topics from configuration
        self.sub_topic = config["subscriber_topics"]["text_input"]
        self.pub_topic = config["publisher_topics"]["pepper_tts_topic"]

        # Initialize the publisher and subscriber
        self.publisher = rospy.Publisher(self.pub_topic, String, queue_size=10)
        self.subscriber = rospy.Subscriber(self.sub_topic, String, self.tts_msg_received)

        rospy.loginfo(
            "TextToSpeechInterface initialized with sub: '%s' and pub: '%s'",
            self.sub_topic,
            self.pub_topic,
        )

    def tts_msg_received(self, msg):
        """
        Called when a TTS message command is received from another node.
        """
        rospy.loginfo("Received TTS command: %s", msg.data)
        self.publish_pepper_tts(msg.data)

    def publish_pepper_tts(self, text):
        """
        Publish the given text to Pepper's TTS topic.
        """
        output_msg = String(data=text)
        self.publisher.publish(output_msg)
        rospy.loginfo("Published TTS output: %s", text)


"""
======================================================================
SensorInterface Class (Handles multiple topics at once)
======================================================================
"""
 #
# class SensorInterface(object):
#     def __init__(self, config):
#         self.sub_topics = config["subscriber_topics"]
#         self.pub_topics = config["publisher_topics"]
#
#         # Create publishers for each topic in the config
#         self.publishers = {
#             'topic1': rospy.Publisher('topic1', String, queue_size=10)
#             'topic2': rospy.Publisher('topic2', String, queue_size=10)
#         }
#
#         # Create subscribers for each topic in the config, refer to callbacks
#         self.subscribers = {
#             'topic1': rospy.Subscriber('topic1', String, self.topic1callback)
#             'topic2': rospy.Subscriber('topic2', String, self.topic2callback)
#         }
#
#         rospy.loginfo(
#             "SensorInterface initialized with subscriber topics: %s and publisher topics: %s",
#             self.sub_topics,
#             self.pub_topics,
#         )
#
#     def topic1callback(self, msg):
#         """
#         A generic callback that could perform processing and then publish to multiple topics.
#         """
#         rospy.loginfo("SensorInterface.topic1callback received message: %s", msg.data)
#         # Example: Publish the same message to all configured output topics.
#         for topic, publisher in self.publishers.items():
#             publisher.publish(msg)
#             rospy.loginfo("Published message to %s: %s", topic, msg.data)
#
#     def topic2callback(self, msg):
#         """
#         Another generic callback lmao
#         """
#         rospy.loginfo("SensorInterface.topic2callback received message: %s", msg.data)
#         message_to_send = String("stuff here lmao")
#         self.publishers['topic2'].publish(message_to_send)

"""
======================================================================
Main execution: Initialize ROS node and start interfaces
======================================================================
"""

if __name__ == "__main__":
    # Initialise ROS Node (name MUST be unique!)
    rospy.init_node("hope_interface")

    # Instantiate TextToSpeechInterface and SensorInterface
    tts_interface = TextToSpeechInterface(CONFIG["text_to_speech_interface"])
    # sensor_interface = SensorInterface(CONFIG["sensor_interface"])

    # Loop until the node is shut down
    rospy.spin()

