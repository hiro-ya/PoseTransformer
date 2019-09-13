#!/usr/bin/env python
# coding: utf-8

from threading import Lock
from argparse import ArgumentParser
import csv

import rospy
from geometry_msgs.msg import PoseStamped

from pose_transformer import pose_to_message, message_to_pose


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-FT', '--from_topic', type=str, default='/vslam_pose/estimated', help='from pose topic')
    parser.add_argument('-TT', '--to_topic', type=str, default='/vslam_pose', help='to pose topic')
    parser.add_argument('-V', '--poses_file_path', type=str, default='../data/poses.csv',
                        help='poses csv file path')
    parser.add_argument('-HZ', '--hz', type=float, default=10, help='publish rate [hz]')
    args = parser.parse_args()

    pose_estimated_array = []
    lock = Lock()

    poses = []
    with open(args.poses_file_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        if header != 't,x,y,z,w,x,y,z'.split(','):
            raise ValueError('poses csv file has wrong header: {}'.format(header))
        for row in reader:
            poses.append(list(map(float, row)))

    rospy.init_node('vslam_pose_estimator', anonymous=True)

    pose_estimated_publisher = rospy.Publisher(
        name=args.to_topic, data_class=PoseStamped, subscriber_listener=None,
        tcp_nodelay=False, latch=False, headers=None, queue_size=0)

    def on_message(data):
        with lock:
            pose_estimated_array.append(message_to_pose(data))
    rospy.Subscriber(args.from_topic, PoseStamped, on_message)

    r = rospy.Rate(args.hz)
    for pose in poses:
        if rospy.is_shutdown():
            break
        pose_estimated_publisher.publish(pose_to_message(pose))
        r.sleep()

    poses_file_path = 'estimated_poses_{}.csv'.format(pose_estimated_array[0][0])
    with open(poses_file_path, 'w') as f:
        writer = csv.writer(f)
        writer.writerow('t,x,y,z,w,x,y,z'.split(','))
        for pose in pose_estimated_array:
            writer.writerow(pose)
    print('{} is saved.'.format(poses_file_path))
