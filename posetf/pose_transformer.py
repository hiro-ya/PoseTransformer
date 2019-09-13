#!/usr/bin/env python
# coding: utf-8

import csv

import numpy as np
from pyquaternion import Quaternion

NEARLY_ZERO = 1e-6
POSE_PAIRS_CSV_HEADER = 'x,y,z,w,x,y,z,ref_x,ref_y,ref_z,ref_w,ref_x,ref_y,ref_z'
TRANSFORM_FACTORS_CSV_HEADER = 'keypoint_x,keypoint_y,keypoint_z,offset_x,offset_y,offset_z,scale_factor,' + \
                               'motion_w,motion_x,motion_y,motion_z,rotation_w,rotation_x,rotation_y,rotation_z'


class PoseTF(object):

    def __init__(self):
        pass

    def __del__(self):
        pass

    @staticmethod
    def load_pose_pairs(file_path):
        pose_pairs = []
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            if header != POSE_PAIRS_CSV_HEADER.split(','):
                raise ValueError('pose pairs csv file has wrong header: {}'.format(header))
            for row in reader:
                row_float = list(map(float, row))
                pose_pairs.append([[row_float[0:3], row_float[3:7]], [row_float[7:10], row_float[10:14]]])
        return pose_pairs

    @classmethod
    def dump_transform_factors(cls, file_path, transform_factors):
        with open(file_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(TRANSFORM_FACTORS_CSV_HEADER.split(','))
            for transform_factor in transform_factors:
                writer.writerow(cls.serialize_transform_factor(transform_factor))

    @classmethod
    def load_transform_factors(cls, file_path):
        transform_factors = []
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            if header != TRANSFORM_FACTORS_CSV_HEADER.split(','):
                raise ValueError('transform factors csv file has wrong header: {}'.format(header))
            for row in reader:
                row_float = list(map(float, row))
                transform_factors.append(cls.deserialize_transform_factor(row_float))
        return transform_factors

    @staticmethod
    def serialize_transform_factor(transform_factor):
        return transform_factor[0].tolist() +\
               transform_factor[1].tolist() +\
               [transform_factor[2]] +\
               transform_factor[3].elements.tolist() +\
               transform_factor[4].elements.tolist()

    @staticmethod
    def deserialize_transform_factor(serialized_transform_factor):
        return [np.array(serialized_transform_factor[0:3]),
                np.array(serialized_transform_factor[3:6]),
                serialized_transform_factor[6],
                Quaternion(serialized_transform_factor[7:11]),
                Quaternion(serialized_transform_factor[11:15])]

    @staticmethod
    def calc_motion_quaternion(v1, v2):
        angle = np.dot(v1, v2)
        if 1 - NEARLY_ZERO < abs(angle):
            return None
        axis = np.cross(v1, v2)
        return Quaternion(angle + np.linalg.norm(v1) * np.linalg.norm(v2), *axis)

    @classmethod
    def generate_transform_factors(cls, pose_pairs):
        positions = np.array(list(map(lambda x: np.array(x[0][0]), pose_pairs)))
        quaternions = list(map(lambda x: Quaternion(x[0][1]), pose_pairs))
        ref_position = np.array(list(map(lambda x: np.array(x[1][0]), pose_pairs)))
        ref_quaternions = list(map(lambda x: Quaternion(x[1][1]), pose_pairs))

        transform_factors = []
        for i in range(0, len(pose_pairs) - 1):
            vslam_vector = positions[i + 1] - positions[i]
            vslam_unit_vector = vslam_vector / np.linalg.norm(vslam_vector)
            current_vector = ref_position[i + 1] - ref_position[i]
            current_unit_vector = current_vector / np.linalg.norm(current_vector)

            key_position = positions[i]
            offset = ref_position[i]
            scale_factor = np.linalg.norm(current_vector) / np.linalg.norm(vslam_vector)
            motion = cls.calc_motion_quaternion(current_unit_vector, vslam_unit_vector).inverse
            rotation = ref_quaternions[i] / quaternions[i]

            transform_factors.append([key_position, offset, scale_factor, motion, rotation])
        return transform_factors

    @staticmethod
    def get_nearest_neighbor_position_index(src_position, key_positions):
        min_distance = None
        nn_key_position_index = None
        for i, key_position in enumerate(key_positions):
            distance = np.linalg.norm(key_position - src_position)
            if min_distance is None or distance < min_distance:
                min_distance = distance
                nn_key_position_index = i
        return nn_key_position_index

    @staticmethod
    def serialize_pose(timestamp, position, rotation):
        return [timestamp] + position.tolist() + rotation.elements.tolist()

    @staticmethod
    def deserialize_pose(pose):
        timestamp = pose[0]
        position = np.array(pose[1:4])
        rotation = Quaternion(pose[4:8])
        return timestamp, position, rotation

    @classmethod
    def transform_pose(cls, src_pose, transform_factors):
        timestamp, src_position, src_rotation = cls.deserialize_pose(src_pose)

        index = cls.get_nearest_neighbor_position_index(src_position, list(map(lambda x: x[0], transform_factors)))
        transform_factor = transform_factors[index]

        src_vector_norm = np.linalg.norm(src_position - transform_factor[0])
        position_transformed = transform_factor[1].copy()
        if transform_factor[3] is not None and NEARLY_ZERO < src_vector_norm:
            unit_src_vector = (src_position - transform_factor[0]) / src_vector_norm
            position_transformed += transform_factor[2] * src_vector_norm * transform_factor[3].rotate(unit_src_vector)
        rotation_transformed = transform_factor[4] * src_rotation

        return cls.serialize_pose(timestamp, position_transformed, rotation_transformed)

