#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-FT', '--from_topic', type=str, default='/image_raw', help='from pose topic')
    parser.add_argument('-TT', '--to_topic', type=str, default='/image_roi', help='to pose topic')
    args = parser.parse_args()

    cvbridge = CvBridge()

    rospy.init_node('image_clipper', anonymous=True)

    image_publisher = rospy.Publisher(
        name=args.to_topic, data_class=Image, subscriber_listener=None,
        tcp_nodelay=False, latch=False, headers=None, queue_size=0)

    def on_message(data):
        cv_image = cvbridge.imgmsg_to_cv2(data, "bgr8")
        proc_data = cvbridge.cv2_to_imgmsg(cv_image[90:-125, 75:-75, :], "bgr8")
        proc_data.header = data.header
        image_publisher.publish(proc_data)
    rospy.Subscriber(args.from_topic, Image, on_message)

    rospy.spin()

