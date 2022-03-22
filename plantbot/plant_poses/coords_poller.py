#!/usr/bin/env python3
import os
import json
from time import sleep
import std_msgs.msg
import kdtree
import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from visualization_msgs.msg import Marker

class CoordinatePollerNode:
    def __init__(self, min_radius = 1):
        self.min_radius = min_radius
        self.poses = kdtree.create(dimensions=3)
        self.poses_q = []
        self.pose_qi = 0
        self.current_pose = None
        self.goal_terminal_state = [
            GoalStatus.ABORTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.PREEMPTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED
        ]
        self.script_path = os.path.dirname(os.path.realpath(__file__))
    
    def coord_poll_one_callback(self, _):
        coord = self.coord_poll_one()
        if coord is not None:
            self.polled.publish(self.__create_stamped_pose(coord))

    def coord_posed_callback(self, data):
        coord = self.__coord_convert(data)
        dists = self.poses.search_nn_dist(coord, self.min_radius)
        if len(dists) == 0:
            rospy.loginfo("Accept new pose (%.2f, %.2f, %.2f)"%coord)
            self.poses.add(coord)
            self.poses_q.append(data)
            self.__add_marker(data)

    def coord_poll_next_callback(self, data):
        state = data.status.status
        self.__save_file(data.status.text)
        if state not in self.goal_terminal_state or not self.do_polling:
            return
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("Goal terminated unexpected!")
            return
        
        os.system('rosrun plantbot spawn_water.py')

        self.coord_poll_one_callback(None)

    def coord_poll_one(self):
        if len(self.poses_q) == 0:
            # rospy.logerr("No pose is being registered!")
            return None
        coord = self.poses_q[self.pose_qi]
        self.pose_qi = (self.pose_qi + 1) % len(self.poses_q)
        self.current_pose = coord
        return coord

    def __add_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "plant_pose"
        marker.id = len(self.poses_q)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.color.r = 0.8
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1
        marker.scale.x = 1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.lifetime = rospy.Duration()
        self.marker.publish(marker)
    
    def start(self):
        rospy.init_node("coord_poller", anonymous=False)
        
        self.min_radius = rospy.get_param("~min_radius", 1)
        self.do_polling = rospy.get_param("~do_polling", False)
        if self.do_polling:
            self.marker = rospy.Publisher("visualization_marker", Marker, queue_size=1)

        self.__read_from_json()
        
        self.polled = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)
        self.sub = rospy.Subscriber("coord_poller/register_goal", Pose, callback=self.coord_posed_callback, queue_size=20)
        self.done = rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback=self.coord_poll_next_callback)
        self.marker = rospy.Publisher("visualization_marker", Marker, queue_size=1)

        # wait for move_base ready before polling coords
        _ = rospy.wait_for_message("/move_base/status", GoalStatusArray, timeout=None)

        if (self.do_polling):
            self.coord_poll_one_callback(None)

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        # self.__save_to_json()

    def __save_file(self, text):
        if text == '' and not self.do_polling:
            print("MAP SAVED")
            rospack = rospkg.RosPack()
            cwd = os.getcwd()
            os.chdir(str(rospack.get_path('plantbot')) + '/maps')
            os.system('rosrun map_server map_saver -f mymap')
            os.chdir(cwd)
            self.__save_to_json()
            self.do_polling = True
            self.coord_poll_one_callback(None)

    def __coord_convert(self, pose):
        return (pose.position.x, pose.position.y, pose.position.z)

    def __create_stamped_pose(self, pose):
        header = std_msgs.msg.Header(frame_id="map")
        return PoseStamped(header=header, pose=pose)

    def __save_to_json(self):
        with open("%s/poses_save.json"%(self.script_path), "w") as f:
            json.dump([self.__pose2json(pose) for pose in self.poses_q], f)
    
    def __read_from_json(self):
        if not os.path.exists("%s/poses_save.json"%(self.script_path)):
            return
        with open("%s/poses_save.json"%(self.script_path), "r") as f:
            if self.do_polling == True:
                poses = json.load(f)
                for obj in poses:
                    self.coord_posed_callback(self.__json2pose(obj))

    def __pose2json(self, pose):
        return {
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            },
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            }
        }

    def __json2pose(self, jobj):
        orientation = jobj['orientation']
        position = jobj['position']
        return Pose(
            position=Point(x=position['x'], y=position['y'], z=position['z']),
            orientation=Quaternion(x=orientation['x'], y=orientation['y'], z=orientation['z'], w=orientation['w'])
        )

if __name__ == "__main__":
    CoordinatePollerNode().start()