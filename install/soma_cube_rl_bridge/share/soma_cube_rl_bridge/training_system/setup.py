from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'soma_cube_training_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS2 User',
    maintainer_email='user@example.com',
    description='Training system for SomaCube RL with Doosan M0609',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_client = soma_cube_rl_training.rl_client:main',
            'train_somacube = soma_cube_rl_training.train_somacube:main',
            'demo_training = soma_cube_rl_training.demo_training:main',
        ],
    },
)