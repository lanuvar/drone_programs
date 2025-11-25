from setuptools import setup

package_name = 'red_drone_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    author='iclal',
    author_email='kacariclal@gmail.com',
    description='Red object search demo with ArduPilot SITL, Gazebo, ROS2, OpenCV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'red_finder = red_drone_demo.red_finder_node:main',
            'drone_brain = red_drone_demo.drone_brain_node:main',
            'mavlink_iface = red_drone_demo.mavlink_iface_node:main',
        ],
    },
)