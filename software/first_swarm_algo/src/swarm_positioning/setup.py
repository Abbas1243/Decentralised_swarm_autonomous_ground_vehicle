from setuptools import setup, find_packages

package_name = 'swarm_positioning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Decentralised swarm positioning around a human in TurtleSim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_tracker_node = swarm_positioning.human_tracker_node:main',
            'robot_agent_node   = swarm_positioning.robot_agent_node:main',
        ],
    },
)