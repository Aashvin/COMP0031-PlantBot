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
    for box in data.bounding_boxes:
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
                    # move_cmd.angular.z = -0.0872665
                    move_cmd.angular.z = -0.01
                elif diff < 0: # box is on left side of middle, move anticlockwise
                    move_cmd.angular.z = 0
                    # move_cmd.angular.z = 0.0872665
                    move_cmd.angular.z = 0.01
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
                    move_cmd.linear.x = 0

            cmd_vel.publish(move_cmd)


            # rospack = rospkg.RosPack()
            msg = rospy.wait_for_message('/odom', Odometry, timeout=None)
            # scan_data = rospy.wait_for_message('/scan', LaserScan, timeout=None)
            # scan_val = scan_data.ranges[0]
            # angle = np.arcsin(msg.pose.pose.orientation.w) * 2
            # print(np.degrees(angle))
            # img = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=None)
            # box_mid = (box.xmax + box.xmin) / 2
            # range_index = int((box_mid / img.width) * 360)
            # print(f"range index: {range_index}")
            # print(f"value: {scan_data.ranges[range_index]}")

            # print(scan_data)
            # print(len(scan_data.ranges))
            # print(msg.pose.pose)
            
            plant_pose = {"x_pos": msg.pose.pose.position.x, "y_pos": msg.pose.pose.position.y, "z_pos": msg.pose.pose.position.z, 
                            "x_rot": msg.pose.pose.orientation.x, "y_rot": msg.pose.pose.orientation.y, "z_rot": msg.pose.pose.orientation.z, "w_rot": msg.pose.pose.orientation.w}
            global rospack
            f = open(rospack.get_path('plantbot') + "/plant_poses/robot_poses.txt", "a")
            f.write(json.dumps(plant_pose) + "\n")
            # f.write(str(msg.pose.pose.position.x) + "\n")
            # f.write(str(msg.pose.pose.position.y) + "\n")
            # f.write(str(msg.pose.pose.position.z) + "\n")
            # f.write(str(msg.pose.pose.orientation.x) + "\n")
            # f.write(str(msg.pose.pose.orientation.y) + "\n")
            # f.write(str(msg.pose.pose.orientation.z) + "\n")
            # f.write(str(msg.pose.pose.orientation.w) + "\n\n")
            f.close()

def main():
    # f = open("robot_poses.txt", "w")
    global exploreStopped 
    exploreStopped = False
    global move_to_plant
    move_to_plant = False
    global rospack
    rospack = rospkg.RosPack()
    f = open(rospack.get_path('plantbot') + "/plant_poses/robot_poses.txt", "w")
    f.close()

    while not rospy.is_shutdown():
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
            cmd_vel.publish(move_cmd)
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes/', BoundingBoxes , callback)
        rospy.spin()


if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
