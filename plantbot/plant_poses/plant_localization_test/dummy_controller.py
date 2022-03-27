#!/usr/bin/env python3

import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Quaternion
import rospy
from tf.transformations import quaternion_about_axis

class DummyController:
    def __init__(self) -> None:
        self.__angular_speed = math.radians(rospy.get_param("~deg_per_sec", 40))
        self.__frequency = min(rospy.get_param("~frequency", 10), 50)
        self.__radius_per_tick = self.__angular_speed / self.__frequency
        self.__rate = rospy.Rate(self.__frequency)

        self.__rotate_robot = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.__model_state = ModelState()
        self.__model_state.reference_frame = "map"
        self.__model_state.model_name = "turtlebot3"
        self.__model_state.pose.position = Point(x=0, y=0, z=0)
        self.__model_state.pose.orientation = Quaternion(x=0, y=0, z=0, w=0)

    def __run(self):
        current_radians = 0
        try:
            rospy.wait_for_service("/gazebo/set_model_state")
            forward = True
            while not rospy.is_shutdown():
                self.__model_state.pose.orientation.w = math.cos(current_radians/2.0)
                self.__model_state.pose.orientation.z = math.sin(current_radians/2.0)
                self.__rotate_robot(self.__model_state)
                if forward:
                    if current_radians >= 0.543:
                        forward = False
                    else:
                        current_radians += self.__radius_per_tick
                else:
                    if current_radians <= -0.543:
                        forward = True
                    else:
                        current_radians -= self.__radius_per_tick
                self.__rate.sleep()
        except rospy.ROSInterruptException:
            pass
    
    @staticmethod
    def start():
        rospy.init_node("dummy_controller", anonymous=True)
        DummyController().__run()

if __name__ == "__main__":
    DummyController.start()