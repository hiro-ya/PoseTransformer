#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
import csv

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-T', '--transform_factors_file_path', type=str,
                        default='./examples/data/transform_factors.csv', help='transform factors csv file path')
    args = parser.parse_args()

    elements = [[], [], [], [], [], [], [], [], []]  # scale_factor, motion_w, motion_x, motion_y, motion_z, rotation_w, rotation_x, rotation_y, rotation_z
    with open(args.transform_factors_file_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            for i, element in enumerate(elements):
                element.append(float(row[6+i]))

    gs = gridspec.GridSpec(2, 2)

    scale_factor = plt.subplot(gs[0, 0])
    motion_xy = plt.subplot(gs[1, 0])
    motion_z = plt.subplot(gs[1, 1])
    rotation = plt.subplot(gs[0, 1])

    scale_factor.plot(list(range(len(elements[0]))), elements[0])
    for element in elements[2:4]:
        motion_xy.plot(list(range(len(element))), element)
    motion_z.plot(list(range(len(elements[4]))), elements[4])
    for element in elements[5:]:
        rotation.plot(list(range(len(element))), element)

    while True:
        try:
            plt.show()
        except UnicodeDecodeError:
            continue
        break
