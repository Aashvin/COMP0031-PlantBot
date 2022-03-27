#!/usr/bin/env python3

import os
from geometry_msgs.msg import Pose
import rospy

class PoseDataCollector:
    def __init__(self) -> None:
        self.__target_sample_num = rospy.get_param("~no_target_sample", 10)
        self.__records = []
        self.__script_path = os.path.dirname(os.path.realpath(__file__))

    def run(self):
        self.__ingress_listener = rospy.Subscriber("/plant_pose_estimate/pose", Pose, callback=self.__ingress_callback, queue_size=10)
        rospy.logwarn("Started, %d"%(self.__target_sample_num))
        rospy.spin()
        self.__save_data_csv()

    def __ingress_callback(self, data: Pose):
        self.__records.append((
            data.position.x, data.position.y, data.position.z,
            data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w
        ))
        rospy.loginfo("Received")
        if len(self.__records) >= self.__target_sample_num:
            rospy.loginfo("Saved data")
            self.__ingress_listener.unregister()
            self.__save_data_csv()

    def __save_data_csv(self):
        with open("%s/poses.csv"%(self.__script_path), "w") as f:
            f.write("px,py,pz,ox,oy,oz,ow\n")
            for elem in self.__records:
                f.write("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n"%elem)

    @staticmethod
    def start():
        rospy.init_node("pose_data_collector", anonymous=True)
        PoseDataCollector().run()

if __name__ == "__main__":
    PoseDataCollector.start()