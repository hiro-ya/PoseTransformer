#!/usr/bin/env python
# coding: utf-8

from threading import Lock
from time import time, sleep
from argparse import ArgumentParser
from copy import deepcopy

import paho.mqtt.client as mqtt
import bson

from posetf import PoseTF


def pose_to_message(pose):
    return bson.dumps({
        'timestamp': pose[0],
        'data': {
            'position': {
                'x': pose[1],
                'y': pose[2],
                'z': pose[3]
            },
            'rotation': {
                'w': pose[4],
                'x': pose[5],
                'y': pose[6],
                'z': pose[7]
            }
        }
    })


def message_to_pose(message):
    pose_message = bson.loads(message.payload)
    return [pose_message['timestamp']] + list(map(lambda k: pose_message['data']['position'][k], ['x', 'y', 'z'])) +\
           list(map(lambda k: pose_message['data']['rotation'][k], ['w', 'x', 'y', 'z']))


def generate_estimated_pose(pose_array, timestamp_offset=None):
    if len(pose_array) < 2:
        return None
    if timestamp_offset is None:
        return None
    current_time = time() + timestamp_offset
    timestamp_1, position_1, rotation_1 = PoseTF.deserialize_pose(pose_array[1])
    timestamp_0, position_0, rotation_0 = PoseTF.deserialize_pose(pose_array[0])
    dt = current_time - timestamp_1
    return PoseTF.serialize_pose(
        current_time, position_1 + (position_1 - position_0) * dt, rotation_1 + (rotation_1 - rotation_0) * dt)


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-H', '--host', type=str, default='localhost', help='mqtt broker host')
    parser.add_argument('-P', '--port', type=int, default=1883, help='mqtt broker port')
    parser.add_argument('-T', '--transform_factors_file_path', type=str,
                        default='./examples/data/transform_factors.csv', help='transform factors csv file path')
    parser.add_argument('-FT', '--from_topic', type=str, default='vslam_pose', help='from pose topic')
    parser.add_argument('-TT', '--to_topic', type=str, default='vslam_pose/estimated', help='to pose topic')
    parser.add_argument('-HZ', '--hz', type=float, default=10, help='publish rate [hz]')
    args = parser.parse_args()

    user_data = {'timestamp_offset': None}
    pose_transformed_array = []
    lock = Lock()

    transform_factors = PoseTF.load_transform_factors(args.transform_factors_file_path)

    def on_message(_client, user_data, message):
        pose = message_to_pose(message)
        pose_transformed = PoseTF.transform_pose(pose, transform_factors)
        with lock:
            user_data['timestamp_offset'] = pose[0] - time()
            pose_transformed_array.append(pose_transformed)
            if 2 < len(pose_transformed_array):
                pose_transformed_array.pop(0)

    try:
        sub_client = mqtt.Client()
        sub_client.user_data_set(user_data)
        sub_client.on_message = on_message
        sub_client.will_set('pose_transformer/lwt', payload=bson.dumps({
            'data': {
                'pubsub': 'sub',
                'topic': args.from_topic
            }
        }), qos=2, retain=False)
        sub_client.connect(args.host, args.port)
        sub_client.subscribe(args.from_topic, qos=2)
        sub_client.loop_start()

        pub_client = mqtt.Client()
        pub_client.will_set('pose_transformer/lwt', payload=bson.dumps({
            'data': {
                'pubsub': 'pub',
                'topic': args.to_topic
            }
        }), qos=2, retain=False)
        pub_client.connect(args.host, args.port)
        pub_client.loop_start()
        dt = 1.0/args.hz
        while True:
            ts = time()
            with lock:
                pose_array = deepcopy(pose_transformed_array)
            pose_estimated = generate_estimated_pose(pose_array, user_data['timestamp_offset'])
            if pose_estimated is not None:
                pub_client.publish(args.to_topic,
                                   payload=pose_to_message(pose_estimated))
            sleep(max(0, dt-(time()-ts)))

    except KeyboardInterrupt:
        pass
