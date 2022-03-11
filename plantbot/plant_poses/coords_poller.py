#!/usr/bin/env python3
import os
import json
import std_msgs
import kdtree
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionResult

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

    def coord_poll_next_callback(self, data):
        state = data.status.status
        rospy.loginfo("Result %s"%(state))
        if state not in self.goal_terminal_state:
            return
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("Goal terminated unexpected!")
        self.coord_poll_one_callback(None)

    def coord_poll_one(self):
        if len(self.poses_q) == 0:
            rospy.logerr("No pose is being registered!")
            return None
        coord = self.poses_q[self.pose_qi]
        self.pose_qi = (self.pose_qi + 1) % len(self.poses_q)
        self.current_pose = coord
        return coord
    
    def start(self):
        rospy.init_node("coord_poller", anonymous=False)
        
        self.min_radius = rospy.get_param("~min_radius", 1)
        self.__read_from_json()
        
        self.polled = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)
        self.sub = rospy.Subscriber("coord_poller/register_goal", Pose, callback=self.coord_posed_callback, queue_size=20)
        self.done = rospy.Subscriber("/move_base/result", MoveBaseActionResult, callback=self.coord_poll_next_callback)
        self.poll_one = rospy.Subscriber("coord_poller/poll_one", std_msgs.msg.Empty, callback=self.coord_poll_one_callback)

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        self.__save_to_json()

    def __coord_convert(self, pose):
        return (pose.position.x, pose.position.y, pose.position.z)

    def __create_stamped_pose(self, pose):
        header = std_msgs.msg.Header(frame_id="map")
        return PoseStamped(header=header, pose=pose)

    def __save_to_json(self):
        with open("%s/poses_save.json"%(self.script_path), "w") as f:
            json.dump([self.__pose2json(pose) for pose in self.poses_q], f)
    
    def __read_from_json(self):
        if not os.path.exists("poses_save.json"):
            return
        with open("%s/poses_save.json"%(self.script_path), "r") as f:
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