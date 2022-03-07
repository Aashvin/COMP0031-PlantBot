import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String
from nav_msgs.msg import Odometry


def callback(data):
    for box in data.bounding_boxes:
#        rospy.loginfo(
#            "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
#                box.xmin, box.xmax, box.ymin, box.ymax
#            )
#        )
        rospy.loginfo("{}, {}".format(box.id, box.Class))
        if box.Class == "pottedplant":
            print("FOUND PLANT")
            msg = rospy.wait_for_message('/odom', Odometry, timeout=None)
            print(msg.pose.pose)
            f = open("robot_poses.txt", "a")
            f.write(str(msg.pose.pose.position.x) + "\n")
            f.write(str(msg.pose.pose.position.y) + "\n")
            f.write(str(msg.pose.pose.position.z) + "\n")
            f.write(str(msg.pose.pose.orientation.x) + "\n")
            f.write(str(msg.pose.pose.orientation.y) + "\n")
            f.write(str(msg.pose.pose.orientation.z) + "\n")
            f.write(str(msg.pose.pose.orientation.w) + "\n\n")
            f.close()

def main():
    f = open("robot_poses.txt", "w")
    f.close()

    while not rospy.is_shutdown():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes/', BoundingBoxes , callback)
        rospy.spin()


if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
