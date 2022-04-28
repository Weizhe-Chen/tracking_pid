#!/usr/bin/env python
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rospy
import tf


class ViconToOdom:

    def __init__(self, object_topic, odom_topic, frame_id):
        # Node initialization
        rospy.init_node("vicon_to_odom_node")
        # Attributes
        self.world_frame = "/vicon/world"
        self.body_frame = object_topic
        self.odom = Odometry()
        self.odom.header.frame_id = frame_id
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.world_frame, self.body_frame,
                                       rospy.Time(), rospy.Duration(5.0))
        # Subscribers and Publishers
        self.sub_trans = rospy.Subscriber(
            object_topic,
            TransformStamped,
            self.trans_callback,
        )
        self.pub_odom = rospy.Publisher(
            odom_topic,
            Odometry,
            queue_size=1,
        )
        # Spinning up
        rospy.spin()

    def trans_callback(self, msg):
        translation, rotation = self.listener.lookupTransform(
            self.world_frame, self.body_frame, rospy.Time())
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = translation[0]
        self.odom.pose.pose.position.y = translation[1]
        self.odom.pose.pose.position.z = translation[2]
        self.odom.pose.pose.orientation.x = rotation[0]
        self.odom.pose.pose.orientation.y = rotation[1]
        self.odom.pose.pose.orientation.z = rotation[2]
        self.odom.pose.pose.orientation.w = rotation[3]
        self.pub_odom.publish(self.odom)
        rospy.loginfo(f"x: {translation[0]: .2f} y: {translation[1]: .2f}")


if __name__ == "__main__":
    try:
        ViconToOdom(object_topic="/vicon/jackal/jackal",
                    odom_topic="/odom",
                    frame_id="/odom")
    except rospy.ROSInterruptException:
        pass
