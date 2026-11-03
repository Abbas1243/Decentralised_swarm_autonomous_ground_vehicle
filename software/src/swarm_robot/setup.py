import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Decentralized boids + consensus swarm control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_agent_node = swarm_robot.swarm_agent_node:main',
            'join_leave_cli = swarm_robot.join_leave_cli:main',
        ],
    },
)
