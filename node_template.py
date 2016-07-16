#!/usr/bin/python

#imports

import rospy

#Messages for Ackermann steering
from ackermann_msgs.msg import *

#Messages for sensors
from sensor_msgs.msg import *

#Common message types for primitive message typesr
from std_msgs.msg import String

class SampleNode:

    def __init__(self):
        self.sample_attribute = 0

        #subscribe to topic - when messages are received, it calls function wuth msgtype as parameter
        rospy.Subscriber("topic_name", String, self.sample_function)

        #Advertise intent to publish to topics, queue_size limits amount of queued messages if subscriber not receiving fart enough
        self.sample_pub = rospy.Publisher("topic_name", String, queue_size=10)
        
        #
