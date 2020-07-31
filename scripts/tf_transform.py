#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from pprint import pprint


def pose_callback(msg):
    global printy
    if len(msg.markers) > 0:
        if tf_listener.frameExists("camera"):
            for m in msg.markers:
                marker_frame = "ar_marker_" + str(m.id)
                if tf_listener.frameExists(marker_frame):
                    t = tf_listener.getLatestCommonTime("camera", marker_frame)
                    tf_listener.waitForTransform("camera", marker_frame, t, rospy.Duration(4.0))
                    (trans1,rot1) = tf_listener.lookupTransform(marker_frame, "camera", t)
                    publish_odom("camera", trans1, rot1)



def publish_odom(frame, trans, rot):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/odom" # i.e. '/odom'
    msg.child_frame_id = frame # i.e. '/base_footprint'
    msg.pose.pose.position = Point(*trans)
    msg.pose.pose.orientation = Quaternion(*rot)
    odom_broadcaster.sendTransform(trans, rot, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)
    odom_pub.publish(msg)


rospy.init_node("ar_track_vis")
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, pose_callback)
tf_listener = tf.TransformListener()

rospy.spin()

