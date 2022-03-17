#!/usr/bin/env python3

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Point, Quaternion
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import tf.transformations
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

    def __yolo_callback(self, data: BoundingBoxes):
        box: BoundingBox
        for box in data.bounding_boxes:
            if box.Class != "pottedplant":
                continue
            x_box = 0.5 * self.cam_hres - ((box.xmin + box.xmax)) / 2
            scan:LaserScan = rospy.wait_for_message("/scan", LaserScan, timeout=None)
            msg = rospy.wait_for_message('/odom', Odometry, timeout=None)
            pose: Pose = msg.pose.pose
            theta = math.atan(2 * x_box * self.__tan_hhfov / self.cam_hres)
            phi = theta if theta >= 0 else scan.angle_max - abs(theta)
            idx = math.ceil(phi / scan.angle_increment)
            d = scan.ranges[idx]
            prev_measure = d
            delta = 0
            total = 2 * self.samples_half_interval + 1
            for i in range(-self.samples_half_interval, self.samples_half_interval):
                measure = scan.ranges[(idx + i) % len(scan.ranges)]
                delta = max(abs(prev_measure - measure), delta)
                prev_measure = measure
                d += measure

            if (d == float("inf")): 
                rospy.logwarn("Not in range!")
                break
            if (delta >= self.max_delta):
                rospy.logwarn("Ignored due to abrupt changes in scan!")
                break
            d = d / total
            if (d < self.min_measure):
                rospy.logwarn("Ignored due to unrealistic scan distance")
                
            R_q = tf.transformations.quaternion_matrix([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])
            v = R_q @ np.array([math.cos(theta), math.sin(theta), 0, 0])
            rospy.logdebug("i=%f, theta=%f, delta=%f, d=%f"%(idx, theta * 180 / math.pi, delta, d))
            coord = v * d + np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z,
                0
            ])

            coord_d = np.linalg.norm(self.__stable_coord - coord)
            if (self.__stable_count > self.__stable_threshold):
                self.__pose_plant(self.__stable_coord, theta, pose)
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

    def __pull_down(self, _ = None):
        if self.__is_down:
            return
        self.__is_down = True
        self.yolo.unregister()

    def __pull_up(self, _ = None):
        if not self.__is_down:
            return
        self.__is_down = False
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
        self.plant_coord = rospy.Publisher("plant_pose_estimate/pose", Pose, queue_size=5)
        rospy.Subscriber("plant_pose_estimate/down", Empty, self.__pull_down, queue_size=2)
        rospy.Subscriber("plant_pose_estimate/up", Empty, self.__pull_up, queue_size=2)
        self.__pull_up()
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('plant_pose_estimate', anonymous=True)
    DistEstimate(
        hfov=rospy.get_param("~horizontal_fov", 62.2),
        cam_hres=rospy.get_param("~img_width", 3280),
        samples_half_interval=rospy.get_param("~half_ray_samples", 1),
        min_measure=rospy.get_param("~min_distance", 0.2),
        coord_stable_threshold=rospy.get_param("~converge_check", 10),
        max_delta=rospy.get_param("~max_scan_delta", 0.2),
    ).start()