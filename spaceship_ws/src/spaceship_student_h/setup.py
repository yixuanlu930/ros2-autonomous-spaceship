from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spaceship_student_h'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yixuan Lu',
    maintainer_email='luguoyixuan@gmail.com',
    description='Controlador autónomo de nave espacial 2D para ROS2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_node = spaceship_student_h.controller_node:main',
        ],
    },
)
