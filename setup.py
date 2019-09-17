#!/usr/bin/env python
# coding: utf-8

from setuptools import setup


setup(
    name='posetf',
    version='0.1.2',
    description='Pose Transformer',
    url='https://github.com/hiro-ya/PoseTransformer',
    author='hiro-ya',
    author_email='dev.hiro.ya@gmail.com',
    license='BSD 2-Clause License',
    packages=['posetf'],
    install_requires=['pyquaternion', 'numpy'],
    extras_require={
        "mqtt": ["paho-mqtt"]
    },
)
