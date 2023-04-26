#! /usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class TagTracker:

    def __init__(self, tag_id_to_track):
        self.tag_id_to_track = tag_id_to_track
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)
        self.image_sub = rospy.Subscriber("/tag_detections_image", Image, self.image_callback)
        self.bridge = CvBridge()

    def tag_callback(self, msg):
        tag_detected = False
        cmd = Twist()

        for detection in msg.detections:
            if detection.id[0] == self.tag_id_to_track:
                tag_detected = True
                x_offset = detection.pose.pose.pose.position.z - 0.5
                z_offset = -detection.pose.pose.pose.position.x

                if x_offset <= 0:  # Stop condition
                    cmd.linear.x = 0
                    cmd.angular.z = 0
                else:
                    # Control robot's linear and angular velocity based on the tag's position
                    cmd.linear.x = 0.2 * x_offset
                    cmd.angular.z = 0.8 * z_offset

        if not tag_detected:
            cmd.linear.x = 0
            cmd.angular.z = 0

        self.pub.publish(cmd)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Tag Detection Image", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == "__main__":
    rospy.init_node("tag_tracker")
    tag_tracker = TagTracker(tag_id_to_track=4)
    rospy.spin()
