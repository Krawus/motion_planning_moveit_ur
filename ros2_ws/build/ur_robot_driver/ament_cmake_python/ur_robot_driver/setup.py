from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_robot_driver',
    version='2.2.6',
    packages=find_packages(
        include=('ur_robot_driver', 'ur_robot_driver.*')),
)
