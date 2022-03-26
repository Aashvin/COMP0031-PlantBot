#!/usr/bin/env python3

import rospy
import os

from std_msgs.msg import String

def callback(data):
    if data.data == "start":
        print("restarting explore node")
        os.system("roslaunch explore_lite explore.launch")
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("start_explore", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()