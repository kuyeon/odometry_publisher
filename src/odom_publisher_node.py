#!/usr/bin/env python3
import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3


rospy.init_node('odometry_publisher')


odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()


x = 0.0
y = 0.0
th = 0.0


vx = 0.1
vy = 0.0
vth = 0.0


current_time = rospy.Time.now()
last_time = rospy.Time.now()


rate = rospy.Rate(1)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    dt = (current_time - last_time).to_sec()
    delta_x = vx * dt

    x += delta_x

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
            )

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    
    odom_pub.publish(odom)

    last_time = current_time
    rate.sleep()
