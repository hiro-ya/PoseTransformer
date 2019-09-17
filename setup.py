#!/usr/bin/env python
# coding: utf-8

from setuptools import setup
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'LICENSE'), encoding='utf-8') as f:
    _license = f.read()

# with open(path.join(here, 'README.md'), encoding='utf-8') as f:
#     long_description = f.read()

setup(
    name='posetf',
    version='0.1.1',
    description='Pose Transformer',
    url='https://github.com/hiro-ya/PoseTransformer',
    author='hiro-ya',
    author_email='dev.hiro.ya@gmail.com',
    license=_license,
    packages=['posetf'],
    install_requires=['pyquaternion', 'numpy'],
    extras_require={
        "mqtt": ["paho-mqtt"]
    },
)
