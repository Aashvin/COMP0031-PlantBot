#!/usr/bin/env python3

from shutil import move
import rospy
import rospkg
import roslaunch
import actionlib
import os

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction

import json
import numpy as np
import time


def callback(data):
    global pub
    for box in data.bounding_boxes:
        # time.sleep(10)
        # print("stopping explore")
        # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # client.wait_for_server()
        # os.system("rosnode kill /explore")
        # client.cancel_all_goals()
        # print("counting to 10")
        # time.sleep(10)
        # print("time up")
        # pub.publish("start")
        # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # client.wait_for_server()
        # os.system("rosnode kill /explore")
        # client.cancel_all_goals()
        # print("counting to 5")
        # time.sleep(5)
        # print("time up")
        # os.system("roslaunch explore_lite explore.launch")
        # print("explore node launched again")
#        rospy.loginfo(
#            "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
#                box.xmin, box.xmax, box.ymin, box.ymax
#            )
#        )
        # print(box.xmax, box.xmin)
        # print(img.width)

        rospy.loginfo("{}, {}".format(box.id, box.Class))
        if box.Class == "pottedplant":
            global exploreStopped
            # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            # client.wait_for_server()
            # client.cancel_all_goals()
            if exploreStopped == False: # kill explore node
                exploreStopped = True
                client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                client.wait_for_server()
                os.system("rosnode kill /explore")
                client.cancel_all_goals()
            print("FOUND PLANT")
            img = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=None)

            box_mid = (box.xmax + box.xmin) / 2
            diff = box_mid - (img.width/2)
            print(f"DIFF: {diff}")
            cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            move_cmd = Twist()
            if abs(diff) > 50:
                if diff > 0: # box is on right side of middle, move clockwise
                    move_cmd.angular.z = 0
                    move_cmd.angular.z = -0.0872665
                    # move_cmd.angular.z = -0.01
                elif diff < 0: # box is on left side of middle, move anticlockwise
                    move_cmd.angular.z = 0
                    move_cmd.angular.z = 0.0872665
                    # move_cmd.angular.z = 0.01
                else:
                    print("should not be here")
            else:
                global move_to_plant
                move_to_plant = True
                move_cmd.angular.z = 0
                scan_data = rospy.wait_for_message('/scan', LaserScan, timeout=None)
                scan_val = scan_data.ranges[0]
                if scan_val > 1:
                    move_cmd.linear.x = 0.2
                else:
                    print("in front of plant...?")
                    global plant_reached
                    plant_reached = True
                    global move_to_plant
                    move_to_plant = False
                    move_cmd.linear.x = 0

            cmd_vel.publish(move_cmd)

            msg = rospy.wait_for_message('/odom', Odometry, timeout=None)

            
            plant_pose = {"x_pos": msg.pose.pose.position.x, "y_pos": msg.pose.pose.position.y, "z_pos": msg.pose.pose.position.z, 
                            "x_rot": msg.pose.pose.orientation.x, "y_rot": msg.pose.pose.orientation.y, "z_rot": msg.pose.pose.orientation.z, "w_rot": msg.pose.pose.orientation.w}
            global rospack
            f = open(rospack.get_path('plantbot') + "/plant_poses/robot_poses.txt", "a")
            f.write(json.dumps(plant_pose) + "\n")
            f.close()

def main():
    global exploreStopped 
    exploreStopped = False
    global pub
    global move_to_plant
    move_to_plant = False
    global plant_reached
    plant_reached = False
    global rospack
    rospack = rospkg.RosPack()
    f = open(rospack.get_path('plantbot') + "/plant_poses/robot_poses.txt", "w")
    f.close()

    while not rospy.is_shutdown():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes/', BoundingBoxes , callback)
        pub = rospy.Publisher('start_explore', String, queue_size=10)
        if move_to_plant == True:
            cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            move_cmd = Twist()
            scan_data = rospy.wait_for_message('/scan', LaserScan, timeout=None)
            scan_val = scan_data.ranges[0]
            if scan_val > 1:
                move_cmd.linear.x = 0.2
            else:
                print("in front of plant...?")
                move_cmd.linear.x = 0
                move_to_plant = False
                plant_reached = True
            cmd_vel.publish(move_cmd)
        if plant_reached == True and exploreStopped == True:
            pub.publish("start")
            plant_reached = False
            exploreStopped = False
        rospy.spin()


if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
