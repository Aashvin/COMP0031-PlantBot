import rospy, tf
import rospkg
import time
import numpy as np
from gazebo_msgs.srv import DeleteModel, SpawnModel, ApplyBodyWrench
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node("spawn_water")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    rospy.wait_for_service("gazebo/apply_body_wrench")

    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    apply_force = rospy.ServiceProxy("gazebo/apply_body_wrench", ApplyBodyWrench)

    msg = rospy.wait_for_message('/odom', Odometry, timeout=None)
    robot_pose = msg.pose.pose.position
    robot_orientation = msg.pose.pose.orientation
    robot_orientation = tf.transformations.euler_from_quaternion([robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w])


    rospack = rospkg.RosPack()
    plantbot_dir = rospack.get_path('plantbot')
    with open(plantbot_dir + "/worlds/models/water_particle/model.sdf", "r") as f:
        product_xml = f.read()

    num_water_particles = 10

    for num in range(num_water_particles):
        item_name = "water_particle_" + str(num)
        item_pose = Pose(Point(x = robot_pose.x, y = robot_pose.y, z = robot_pose.z + 0.2),   Quaternion(x = 0, y = 0, z = 0, w = 1))
        spawn_model(item_name, product_xml, "", item_pose, "world")

        x_force = np.cos(robot_orientation[2])
        y_force = np.sin(robot_orientation[2])
        wrench = Wrench(force=Vector3(x = x_force, y = y_force, z = 1.5))
        duration = rospy.Duration(secs = 0.1, nsecs = 0)
        apply_force(body_name = item_name + "::link", wrench = wrench, duration = duration)


    for num in range(num_water_particles):
        item_name = "water_particle_" + str(num)
        delete_model(item_name)
