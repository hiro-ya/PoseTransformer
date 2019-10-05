#!/usr/bin/env python
# coding: utf-8

import csv
from argparse import ArgumentParser

import rospy
from geometry_msgs.msg import PoseStamped


def stamp_to_timestamp(ros_header_stamp):
    return ros_header_stamp.secs + 1e-9 * ros_header_stamp.nsecs


def append_pose(poses, data):
    poses.append([
        stamp_to_timestamp(data.header.stamp), 
        data.pose.position.x,
        data.pose.position.y,
        data.pose.position.z,
        data.pose.orientation.w,
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z
    ])

        
if __name__ == '__main__':
    """
    subscribe reference_poses and poses.
    save as csv file.
    
    $ roscore
    $ rosparam set use_sim_time true
    $ rosbag play {} --topics {} --clock
    """
    parser = ArgumentParser()
    parser.add_argument('-R', '--reference_pose_topic', type=str, default='/current_pose',
                        help='reference pose topic')
    parser.add_argument('-P', '--pose_topic', type=str, default='/openvslam_pose',
                        help='pose topic')
    args = parser.parse_args()

    rospy.init_node('pose_dumper', anonymous=True)

    reference_poses = []

    def callback_reference_pose(data):
        append_pose(reference_poses, data)

    rospy.Subscriber(args.reference_pose_topic, PoseStamped, callback_reference_pose)

    poses = []

    def callback_pose(data):
        append_pose(poses, data)

    rospy.Subscriber(args.pose_topic, PoseStamped, callback_pose)

    rospy.spin()

    reference_poses_file_path = "reference_poses_{}.csv".format(reference_poses[0][0])
    with open(reference_poses_file_path, "w") as f:
        writer = csv.writer(f)
        writer.writerow('t,x,y,z,w,x,y,z'.split(','))
        for reference_pose in reference_poses:
            writer.writerow(reference_pose)
    print('{} is saved.'.format(reference_poses_file_path))
    poses_file_path = "poses_{}.csv".format(reference_poses[0][0])
    with open(poses_file_path, "w") as f:
        writer = csv.writer(f)
        writer.writerow('t,x,y,z,w,x,y,z'.split(','))
        for pose in poses:
            writer.writerow(pose)
    print('{} is saved.'.format(poses_file_path))

