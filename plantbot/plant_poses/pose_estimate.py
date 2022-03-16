#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import tf.transformations
import numpy as np
import rospy
import math

class DistEstimate:
    def __init__(self, hfov = 62.2, cam_hres = 3280, samples_half_interval = 5):
        self.__recent_scan = None
        self.plant_found = False
        self.hfov = math.radians(hfov)
        self.__tan_hhfov = math.tan(0.5 * hfov)
        self.cam_hres = cam_hres
        self.samples_half_interval = samples_half_interval

    def yolo_callback(self, data: BoundingBoxes):
        box: BoundingBox
        for box in data.bounding_boxes:
            if box.Class != "pottedplant":
                continue
            x_box = (0.5 * self.cam_hres - (box.xmin + box.xmax)) / 2
            msg = rospy.wait_for_message('/odom', Odometry, timeout=None)
            scan:LaserScan = rospy.wait_for_message("/scan", LaserScan, timeout=None)
            pose: Pose = msg.pose.pose
            theta = math.atan(2 * x_box * self.__tan_hhfov / self.cam_hres)
            phi = theta if theta >= 0 else scan.angle_max - abs(theta)
            idx = math.floor(phi / scan.angle_increment)
            d = 0
            for i in range(-self.samples_half_interval, self.samples_half_interval):
                d += scan.ranges[(idx + i) % len(scan.ranges)]
            d = d / (2 * self.samples_half_interval)
            if (d == float("inf")): 
                rospy.logwarn("Not in range!")
                continue
                
            R_q = tf.transformations.quaternion_matrix([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])
            v = R_q @ np.array([math.cos(theta), math.sin(theta), 0, 0])
            rospy.loginfo("i=%f, theta=%f"%(idx, theta * 180 / math.pi))
            coord = v * d + np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
                0
            ])
            rospy.loginfo("FOUND PLANT: (%f, %f, %f)"%(coord[0], coord[1], coord[2]))
    def start(self):
        rospy.init_node('plant_pose_estimate', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes/', BoundingBoxes , self.yolo_callback, queue_size=5)
        rospy.spin()

if __name__ == "__main__":
    DistEstimate().start()