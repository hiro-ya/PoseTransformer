#!/usr/bin/env python
# coding: utf-8

import csv
from argparse import ArgumentParser

import numpy as np
from posetf import POSE_PAIRS_CSV_HEADER


if __name__ == '__main__':
    """
        pairing pose synchronized with reference_pose time.
        save as csv file.
    """
    parser = ArgumentParser()
    parser.add_argument('-C', '--reference_poses_file_path', type=str, default='./examples/data/reference_poses.csv',
                        help='reference_poses csv file path')
    parser.add_argument('-V', '--poses_file_path', type=str, default='./examples/data/poses.csv',
                        help='poses csv file path')
    parser.add_argument('-P', '--pose_pairs_file_path', type=str, default='./examples/data/pose_pairs.csv',
                        help='pose_pairs csv file path')
    parser.add_argument('-TT', '--time_tolerance', type=float, default=1e-2,
                        help='time tolerance')
    parser.add_argument('-PT', '--position_tolerance', type=float, default=1.0,
                        help='reference position tolerance')
    args = parser.parse_args()

    reference_poses = []
    with open(args.reference_poses_file_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        if header != 't,x,y,z,w,x,y,z'.split(','):
            raise ValueError('poses csv file has wrong header: {}'.format(header))
        for row in reader:
            reference_poses.append(list(map(float, row)))

    poses = []
    with open(args.poses_file_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        if header != 't,x,y,z,w,x,y,z'.split(','):
            raise ValueError('poses csv file has wrong header: {}'.format(header))
        for row in reader:
            poses.append(list(map(float, row)))

    pose_pairs = []
    for pose in poses:
        nearest_reference_pose = min(reference_poses, key=lambda x: abs(pose[0]-x[0]))
        dt = abs(pose[0] - nearest_reference_pose[0])
        if dt < args.time_tolerance:
            if len(pose_pairs) == 0:
                pose_pairs.append(pose[1:]+nearest_reference_pose[1:])
            else:
                d = np.linalg.norm(np.array(nearest_reference_pose[1:4])-np.array(pose_pairs[-1][7:10]))
                if args.position_tolerance < d:
                    pose_pairs.append(pose[1:]+nearest_reference_pose[1:])
        else:
            print(dt)

    with open(args.pose_pairs_file_path, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(POSE_PAIRS_CSV_HEADER.split(','))
        for pose_pair in pose_pairs:
            writer.writerow(pose_pair)
