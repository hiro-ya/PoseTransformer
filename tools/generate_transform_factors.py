#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
from posetf import PoseTF


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-P', '--pose_pairs_file_path', type=str,
                        default='./examples/data/pose_pairs.csv', help='pose_pairs csv file path')
    parser.add_argument('-T', '--transform_factors_file_path', type=str,
                        default='./examples/data/transform_factors.csv', help='transform_factors csv file path')
    args = parser.parse_args()

    pose_pairs = PoseTF.load_pose_pairs(args.pose_pairs_file_path)
    transform_factors = PoseTF.generate_transform_factors(pose_pairs)
    PoseTF.dump_transform_factors(args.transform_factors_file_path, transform_factors)
