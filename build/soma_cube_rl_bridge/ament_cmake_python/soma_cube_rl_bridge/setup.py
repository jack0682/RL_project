from setuptools import find_packages
from setuptools import setup

setup(
    name='soma_cube_rl_bridge',
    version='1.0.0',
    packages=find_packages(
        include=('soma_cube_rl_bridge', 'soma_cube_rl_bridge.*')),
)
