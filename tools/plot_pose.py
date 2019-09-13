#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
import csv

import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt

from posetf import PoseTF

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-V', '--poses_file_path', type=str, default='./examples/data/poses.csv',
                        help='poses csv file path')
    args = parser.parse_args()

    x, y, x_ref, y_ref = [], [], [], []
    dx, dy = [], []
    with open(args.poses_file_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
            x_ref.append(float(row[7]))
            y_ref.append(float(row[8]))
            _, position, _ = PoseTF.deserialize_pose([0.0]+list(map(float, row[0:7])))
            _, position_ref, rotation_ref = PoseTF.deserialize_pose([0.0]+list(map(float, row[7:14])))
            dp = position_ref - position
            dx.append(dp[0])
            dy.append(dp[1])

    hypot_median = np.median(list(map(lambda x: np.hypot(*x), zip(dx, dy))))
    print(hypot_median)
    for i in range(len(x)):
        if hypot_median < np.hypot(dx[i], dy[i]):
            plt.arrow(x[i], y[i], dx[i], dy[i],
                      width=0.25, head_width=1.5, head_length=5, length_includes_head=True,
                      color='red')

    plt.scatter(np.array(x_ref), np.array(y_ref), c='green')
    plt.scatter(np.array(x), np.array(y), c='blue')

    while True:
        try:
            plt.show()
        except UnicodeDecodeError:
            continue
        break
