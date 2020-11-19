#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    version='0.1.0',
    name='menge_gym',
    packages=['envs', 'envs.utils'],
    package_dir={'': 'src'},
    author='Julian Kunze',
    author_email='julian-kunze@gmx.de',
)

setup(
    install_requires=[
        'gym',
        'numpy',
        'scipy',
        'scikit-learn',
        'filterpy'
    ],
    extras_require={
        'test': [
            'pylint',
            'pytest',
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    **setup_args
)
