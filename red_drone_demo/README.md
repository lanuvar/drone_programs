# Red Drone Demo

## Overview
The Red Drone Demo project is designed to demonstrate the capabilities of a drone system that can detect red objects and perform various operations. This project utilizes the Robot Operating System (ROS) for managing the drone's functionalities.

## Project Structure
The project consists of the following key components:

- **package.xml**: Contains metadata about the package, including its name, version, description, maintainers, and dependencies.
- **setup.py**: A script for setting up the package, including information about the package and its dependencies.
- **setup.cfg**: Configuration settings for the package.
- **resource/red_drone_demo**: Directory for additional resources or data files needed by the package.
- **launch/red_search.launch.py**: Launch file for starting ROS nodes and setting parameters.
- **red_drone_demo/**: Contains the main package code, including:
  - `__init__.py`: Marks the directory as a Python package.
  - `drone_brain_node.py`: Implements the logic for the drone's operations.
  - `mavlink_iface_node.py`: Interfaces with MAVLink for communication with the drone.
  - `red_finder_node.py`: Responsible for detecting red objects or signals.
- **tests/test_nodes.py**: Contains unit tests for the nodes in the package.

## Installation
To set up the project, follow these steps:

1. Ensure you have the necessary dependencies installed as specified in `package.xml` and `setup.py`.
2. Run the following command to install the package:
   ```
   python setup.py install
   ```

## Running the Project
To launch the ROS nodes defined in the launch file, use the following command:
```
ros2 launch red_drone_demo red_search.launch.py
```

## Contributing
Contributions to the project are welcome. Please feel free to submit issues or pull requests.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.