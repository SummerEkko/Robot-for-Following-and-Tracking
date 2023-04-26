#! /usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import tf

def odom_callback(odom):
    global path_pub, path
    this_pose_stamped = PoseStamped()
    this_pose_stamped.pose.position.x = odom.pose.pose.position.x
    this_pose_stamped.pose.position.y = odom.pose.pose.position.y
    this_pose_stamped.pose.position.z = odom.pose.pose.position.z + 0.1
    this_pose_stamped.pose.orientation = odom.pose.pose.orientation
    this_pose_stamped.header.stamp = rospy.Time.now()
    this_pose_stamped.header.frame_id = "odom"
    path.poses.append(this_pose_stamped)
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "odom"
    path_pub.publish(path)
    print("path_pub:")
    print("odom %.3lf %.3lf %.3lf" % (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z))

def main():
    global path_pub, path
    rospy.init_node('showpath_odom')
    path_pub = rospy.Publisher('trajectory_odom', Path, queue_size=10)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    path = Path()
    rate = rospy.Rate(50)
   
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
