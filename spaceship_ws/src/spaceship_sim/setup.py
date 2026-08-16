from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spaceship_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Profesor',
    maintainer_email='profesor@universidad.es',
    description='Simulador 2D nave espacial',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ship_simulator  = spaceship_sim.ship_simulator:main',
            'rviz_publisher  = spaceship_sim.rviz_publisher:main',
        ],
    },
)
