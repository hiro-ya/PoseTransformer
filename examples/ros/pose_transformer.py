#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from threading import Lock
from math import modf
from argparse import ArgumentParser

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped

from posetf import PoseTF

POSE_ARRAY_SIZE = 5


def rosstamp_to_timestamp(rosstamp):
    return rosstamp.secs + 1e-9 * rosstamp.nsecs


def timestamp_to_rosstamp(timestamp):
    secs_f, secs = modf(timestamp)
    return rospy.Time(int(secs), int(1e9*secs_f))


def pose_to_message(pose):
    data = PoseStamped()
    data.header.frame_id = '/map'
    data.header.stamp = timestamp_to_rosstamp(pose[0])
    data.pose.position.x = pose[1]
    data.pose.position.y = pose[2]
    data.pose.position.z = pose[3]
    data.pose.orientation.w = pose[4]
    data.pose.orientation.x = pose[5]
    data.pose.orientation.y = pose[6]
    data.pose.orientation.z = pose[7]
    return data


def message_to_pose(data):
    return [
        rosstamp_to_timestamp(data.header.stamp),
        data.pose.position.x,
        data.pose.position.y,
        data.pose.position.z,
        data.pose.orientation.w,
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z
    ]


def generate_estimated_pose(pose_array):
    if len(pose_array) < POSE_ARRAY_SIZE:
        return None
    current_time = rosstamp_to_timestamp(rospy.Time.now())
    poses = list(map(PoseTF.deserialize_pose, pose_array))
    predict_positions = list(map(lambda x: x[1] + (poses[-1][1] - x[1]) * (current_time - x[0]), poses))
    predict_rotations = list(map(lambda x: (x[2] + (poses[-1][2] - x[2]) * (current_time - x[0])).elements, poses))
    position = np.median(predict_positions, axis=0)
    rotation = np.median(predict_rotations, axis=0)
    return [current_time] + position.tolist() + rotation.tolist()


if __name__ == '__main__':
    """
    subscribe openvslam_pose and publish matched pose
    """
    parser = ArgumentParser()
    parser.add_argument('-T', '--transform_factors_file_path', type=str,
                        default='./examples/data/transform_factors.csv', help='transform factors csv file path')
    parser.add_argument('-FT', '--from_topic', type=str, default='/vslam_pose', help='from pose topic')
    parser.add_argument('-TT', '--to_topic', type=str, default='/vslam_pose/estimated', help='to pose topic')
    parser.add_argument('-HZ', '--hz', type=float, default=10, help='publish rate [hz]')
    args = parser.parse_args()

    while True:
        pose_transformed_array = []
        lock = Lock()

        transform_factors = PoseTF.load_transform_factors(args.transform_factors_file_path)

        rospy.init_node('posetf', anonymous=True)

        pose_estimated_publisher = rospy.Publisher(
            name=args.to_topic, data_class=PoseStamped, subscriber_listener=None,
            tcp_nodelay=False, latch=False, headers=None, queue_size=0)

        def on_message(data):
            pose = message_to_pose(data)
            pose_transformed = PoseTF.transform_pose(pose, transform_factors)
            with lock:
                pose_transformed_array.append(pose_transformed)
                if POSE_ARRAY_SIZE < len(pose_transformed_array):
                    pose_transformed_array.pop(0)
        rospy.Subscriber(args.from_topic, PoseStamped, on_message)

        try:
            r = rospy.Rate(args.hz)
            while not rospy.is_shutdown():
                with lock:
                    pose_data_array = deepcopy(pose_transformed_array)
                pose_estimated = generate_estimated_pose(pose_data_array)
                if pose_estimated is not None:
                    pose_estimated_publisher.publish(pose_to_message(pose_estimated))
                r.sleep()
        except rospy.ROSInterruptException as e:
            print(e)
            if e[0] == 'ROS time moved backwards':
                print('retry')
            else:
                break
