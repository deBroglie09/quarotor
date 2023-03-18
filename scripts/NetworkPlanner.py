#! /usr/bin/env python3
# coding=utf8
 
import rospy
from std_msgs.msg import  String
 
if __name__ == "__main__":
    rospy.init_node("pub_message")
    pub = rospy.Publisher("ros_message",String,queue_size=10)
    message = String()
    rate = rospy.Rate(1)
    count = 0
    while not rospy.is_shutdown():
        message.data = "pub message to sub " +str(count)
        pub.publish(message)
        rospy.loginfo("pub message it is : %s",message.data)
        count+=1
        rate.sleep()