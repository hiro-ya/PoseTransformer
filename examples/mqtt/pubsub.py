#!/usr/bin/env python
# coding: utf-8

from threading import Lock
from time import time, sleep
from argparse import ArgumentParser
import csv

import paho.mqtt.client as mqtt

from pose_transformer import pose_to_message, message_to_pose


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-H', '--host', type=str, default='localhost', help='mqtt broker host')
    parser.add_argument('-P', '--port', type=int, default=1883, help='mqtt broker port')
    parser.add_argument('-FT', '--from_topic', type=str, default='vslam_pose/estimated', help='from pose topic')
    parser.add_argument('-TT', '--to_topic', type=str, default='vslam_pose', help='to pose topic')
    parser.add_argument('-V', '--poses_file_path', type=str, default='../data/poses.csv',
                        help='poses csv file path')
    parser.add_argument('-HZ', '--hz', type=float, default=30, help='publish rate [hz]')
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

    def on_message(_client, _user_data, message):
        with lock:
            pose_estimated_array.append(message_to_pose(message))

    sub_client = mqtt.Client()
    sub_client.on_message = on_message
    sub_client.connect(args.host, args.port)
    sub_client.subscribe(args.from_topic, qos=2)
    sub_client.loop_start()

    pub_client = mqtt.Client()
    pub_client.connect(args.host, args.port)
    pub_client.loop_start()
    dt = 1.0/args.hz
    for pose in poses:
        ts = time()
        pub_client.publish(args.to_topic, payload=pose_to_message(pose))
        sleep(max(0, dt-(time()-ts)))

    poses_file_path = 'estimated_poses_{}.csv'.format(pose_estimated_array[0][0])
    with open(poses_file_path, 'w') as f:
        writer = csv.writer(f)
        writer.writerow('t,x,y,z,w,x,y,z'.split(','))
        for pose in pose_estimated_array:
            writer.writerow(pose)
    print('{} is saved.'.format(poses_file_path))
