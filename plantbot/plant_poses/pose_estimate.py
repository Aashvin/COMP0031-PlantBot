#!/usr/bin/env python3

from time import sleep
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Point, Quaternion
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import tf.transformations
import tf
import numpy as np
import rospy
import math

class DistEstimate:
    def __init__(self, hfov = 62.2, cam_hres = 3280, samples_half_interval = 0, coord_stable_threshold = 15, max_delta=0.2, min_measure=0.2):
        self.hfov = math.radians(hfov)
        self.__tan_hhfov = math.tan(0.5 * hfov)
        self.cam_hres = cam_hres
        self.samples_half_interval = samples_half_interval
        self.max_delta = max_delta
        self.min_measure = min_measure
        self.__stable_coord = np.array([0.,0.,0.,0.])
        self.__stable_count = 0
        self.__stable_threshold = coord_stable_threshold
        self.__is_down = True
        self.__is_plant_found = False
        self.__odom_pose = None
        self.__scan = None

    def __odom_update_callback(self, data: Odometry):
        if self.__is_plant_found:
           return
        self.__odom_pose = data.pose.pose

    def __scan_update_callback(self, data: LaserScan):
        if self.__is_plant_found:
           return
        self.__scan = data

    def __yolo_callback(self, data: BoundingBoxes):
        box: BoundingBox
        for box in data.bounding_boxes:
            if box.Class != "pottedplant":
                continue
            self.__is_plant_found = True

            # calculate the bounding box's offset angle from the optical axis
            x_box = 0.5 * self.cam_hres - ((box.xmin + box.xmax)) / 2
            theta = math.atan(2 * x_box * self.__tan_hhfov / self.cam_hres)
            c_o = np.array([math.cos(theta), math.sin(theta), 0, 0])

            # apply a transformation from camera frame to scanner frame
            v_o = (self.cam_R @ c_o + self.cam_V)

            # calculate which scan line to use...
            phi = math.atan(v_o[1] / v_o[0])
            phi = phi if phi >= 0 else self.__scan.angle_max - abs(phi)
            idx = round(phi / self.__scan.angle_increment)

            # use additional ray to approximate
            d = self.__scan.ranges[idx]
            prev_measure = d
            delta = 0
            total = 2 * self.samples_half_interval + 1
            for i in range(-self.samples_half_interval, self.samples_half_interval):
                measure = self.__scan.ranges[(idx + i) % len(self.__scan.ranges)]
                delta = max(abs(prev_measure - measure), delta)
                prev_measure = measure
                d += measure

            # sanity check...
            if (d == float("inf")): 
                rospy.logwarn("Not in range!")
                break
            if (delta >= self.max_delta):
                rospy.logwarn("Ignored due to abrupt changes in scan!")
                break

            # offset to make space
            d = d / total - 0.5
            if (d < self.min_measure):
                rospy.logwarn("Ignored due to unrealistic scan distance")
                break
            
            # transform to world coordinate!
            R_q = tf.transformations.quaternion_matrix([
                self.__odom_pose.orientation.x,
                self.__odom_pose.orientation.y,
                self.__odom_pose.orientation.z,
                self.__odom_pose.orientation.w
            ])
            v = R_q @ (self.base_R @ v_o + self.base_V)
            rospy.logdebug("i=%f, theta=%f, delta=%f, d=%f"%(idx, theta * 180 / math.pi, delta, d))
            coord = v * d + np.array([
                self.__odom_pose.position.x,
                self.__odom_pose.position.y,
                self.__odom_pose.position.z,
                0
            ])
            
            # stablizing the coordinates
            coord_d = np.linalg.norm(self.__stable_coord - coord)
            if (self.__stable_count > self.__stable_threshold):
                self.__pose_plant(self.__stable_coord, theta, self.__odom_pose)
                rospy.loginfo("PLANT STABLED: (%f, %f, %f)"%(
                    self.__stable_coord[0], self.__stable_coord[1], self.__stable_coord[2]))
                self.__stable_count = 1
            
            if (coord_d >= 1):
                self.__stable_coord = coord
                self.__stable_count = 1
            else:
                self.__stable_count += 1
                self.__stable_coord += (coord - self.__stable_coord) / self.__stable_count
            break
        self.__is_plant_found = False

    def __pull_down(self, _ = None):
        if self.__is_down:
            return
        self.__is_down = True
        self.yolo.unregister()
        self.scan.unregister()
        self.odom.unregister()

    def __pull_up(self, _ = None):
        if not self.__is_down:
            return
        self.__is_down = False
        self.scan = rospy.Subscriber('/scan', LaserScan , self.__scan_update_callback, queue_size=2)
        self.odom = rospy.Subscriber('/odom', Odometry , self.__odom_update_callback, queue_size=2)
        sleep(1)
        self.yolo = rospy.Subscriber('/darknet_ros/bounding_boxes/', BoundingBoxes , self.__yolo_callback, queue_size=2)

    def __pose_plant(self, coord, angle, robot_pose: Pose):
        _q = tf.transformations.quaternion_about_axis(angle, (0,0,1))
        _q_prime = np.array([
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w
        ])
        qq_prime = tf.transformations.quaternion_multiply(_q, _q_prime)
        pose = Pose(
            position=Point(
                x=coord[0],
                y=coord[1],
                z=coord[2]
            ),
            orientation=Quaternion(x=qq_prime[0], y=qq_prime[1], z=qq_prime[2], w=qq_prime[3])
        )
        self.plant_coord.publish(pose)

    
    def start(self):
        self.plant_coord = rospy.Publisher("/coord_poller/register_goal", Pose, queue_size=5)
        rospy.Subscriber("plant_pose_estimate/down", Empty, self.__pull_down, queue_size=2)
        rospy.Subscriber("plant_pose_estimate/up", Empty, self.__pull_up, queue_size=2)
        retry = 20
        tf_listener = tf.TransformListener()
        while retry > 0:
            try:
                (trans,rot) = tf_listener.lookupTransform('/base_scan', '/camera_link', rospy.Time(0))
                (transb,rotb) = tf_listener.lookupTransform('/base_link', '/base_scan', rospy.Time(0))
                self.cam_V = np.array([trans[0], trans[1], trans[2], 0])
                self.base_V = np.array([transb[0], transb[1], transb[2], 0])
                self.cam_R = tf.transformations.quaternion_matrix(rot)
                self.base_R = tf.transformations.quaternion_matrix(rotb)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Fail to acquire transformation info! Retry in 1 sec")
                retry-=1
                sleep(1)
        if (retry == 0):
            rospy.logfatal("Unable to  acquire transformation info after 20 retries! Terminating...")
            return
        self.__pull_up()
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('plant_pose_estimate', anonymous=True)
    DistEstimate(
        hfov=rospy.get_param("~horizontal_fov", 62.2),
        cam_hres=rospy.get_param("~img_width", 3280),
        samples_half_interval=rospy.get_param("~half_ray_samples", 2),
        min_measure=rospy.get_param("~min_distance", 0.2),
        coord_stable_threshold=rospy.get_param("~converge_check", 10),
        max_delta=rospy.get_param("~max_scan_delta", 0.1),
    ).start()