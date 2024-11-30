"""webots_ros2 package setup file."""

from setuptools import setup

package_name = 'webots_ros2_tesla'
data_files = []

# Add package resources for ament
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

# Add launch files
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))

# Add world files
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/tesla_world.wbt', 'worlds/.tesla_world.wbproj',
]))

# Add robot description file (URDF)
data_files.append(('share/' + package_name + '/resource', [
    'resource/tesla_webots.urdf'
]))

# Add configuration files for slam_toolbox
data_files.append(('share/' + package_name + '/config', [
    'config/slam_toolbox_config.yaml'
]))

# Add package.xml
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='2023.1.3',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tesla ROS2 interface for Webots with lidar, odometry, and SLAM support.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follower = webots_ros2_tesla.lane_follower:main',
            'lidar_publisher = webots_ros2_tesla.lidar_publisher:main',
            'odometry_publisher = webots_ros2_tesla.odometry_publisher:main',  # Added odometry publisher script
            'kf_node = webots_ros2_tesla.kf_node:main',  # Added KF node
            'obstacle_avoidance = tesla_autopilot.obstacle_avoidance_node:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
