#!/usr/bin/env python

import rospy

import math
import PyKDL
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from cuda_amcl.msg import vegvisir_msg

last_received_pose = Point()
curr_received_pose = Point()
final_received_pose = Point()

class PoseSetter(rospy.SubscribeListener):
    def __init__(self, pose, stamp, publish_time):
        self.pose = pose
        self.stamp = stamp
        self.publish_time = publish_time
        self.p =PoseWithCovarianceStamped()
        self.p.header.frame_id = "map"
        self.p.header.stamp = self.stamp
        self.p.pose.pose.position.x = self.pose[0]
        self.p.pose.pose.position.y = self.pose[1]
        (
            self.p.pose.pose.orientation.x,
            self.p.pose.pose.orientation.y,
            self.p.pose.pose.orientation.z,
            self.p.pose.pose.orientation.w,
        ) = PyKDL.Rotation.RPY(0, 0, self.pose[2]).GetQuaternion()
        self.p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        self.p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        self.p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0

    # def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    #     p = PoseWithCovarianceStamped()
    #     p.header.frame_id = "map"
    #     p.header.stamp = self.stamp
    #     p.pose.pose.position.x = self.pose[0]
    #     p.pose.pose.position.y = self.pose[1]
    #     (
    #         p.pose.pose.orientation.x,
    #         p.pose.pose.orientation.y,
    #         p.pose.pose.orientation.z,
    #         p.pose.pose.orientation.w,
    #     ) = PyKDL.Rotation.RPY(0, 0, self.pose[2]).GetQuaternion()
    #     p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
    #     p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
    #     p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0
    #     # wait for the desired publish time
    #     rospy.loginfo("pub")
    #     while rospy.get_rostime() < self.publish_time:
    #         rospy.sleep(0.01)
    #     peer_publish(p)

def reset_pose_callback(data):
    global final_received_pose
    final_received_pose = data

do_once=1
def veg_callback(data):
    global curr_received_pose
    global last_received_pose
    global final_received_pose

    global do_once

    curr_received_pose = data.amcl_pose.pose.pose.position

    if do_once:
        do_once=0
        last_received_pose = curr_received_pose

    t_stamp = rospy.Time()
    t_publish = rospy.Time()
    # rospy.loginfo(
    #     "Received pose {} with stamp {} at {}".format(
    #         data, t_stamp.to_sec(), t_publish.to_sec()
    #     )
    # )

    distance = math.sqrt((curr_received_pose.x-last_received_pose.x)**2 + (curr_received_pose.y - last_received_pose.y)**2)
    rospy.loginfo("currx={},y={}.z={}".format(curr_received_pose.x,curr_received_pose.y,curr_received_pose.z))

    # rospy.loginfo(distance)
    if distance > 3:
        rospy.loginfo("true")
        rospy.loginfo("x={},y={}.z={}".format(final_received_pose.x,final_received_pose.y,final_received_pose.z))
        p=PoseSetter([final_received_pose.x/1000,final_received_pose.y/1000,final_received_pose.z], stamp=t_stamp, publish_time=t_publish)
        pub = rospy.Publisher(
            "initialpose",
            PoseWithCovarianceStamped,
            queue_size=1,
        )
        pub.publish(p.p)
    else:
        last_received_pose = curr_received_pose
        pub = rospy.Publisher("amcl_output", vegvisir_msg, queue_size=10)
        pub.publish(data)
        rospy.loginfo("false")


if __name__ == "__main__":
    rospy.init_node("reset_pose", anonymous=True)
    rospy.Subscriber("vegvisir_topic", vegvisir_msg, veg_callback)
    rospy.Subscriber("final_pose", Point, reset_pose_callback)

    rospy.spin()
