from setuptools import find_packages
from setuptools import setup

setup(
    name='very_simple_robot_simulator',
    version='0.0.0',
    packages=find_packages(
        include=('very_simple_robot_simulator', 'very_simple_robot_simulator.*')),
)
