
<div align="center">

# AMCAF

A vehicle has to navigate through a lane-less scenario with vehicles moving not in a structured manner. A framework for avoiding collision with ackermann steering is to be made along with making sure the vehicle movement is smooth and non-violating traffic rules.

[![Category](https://img.shields.io/badge/Category-ROS2%20%2F%20Robotics-blue)](#)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![e-Yantra](https://img.shields.io/badge/e--Yantra-IIT%20Bombay-orange)](https://www.e-yantra.org)

</div>

---

## Table of Contents

- [About](#about)
- [Prerequisites](#prerequisites)
- [Getting Started](#getting-started)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Team](#team)
- [License](#license)

---

## About

A vehicle has to navigate through a lane-less scenario with vehicles moving not in a structured manner. A framework for avoiding collision with ackermann steering is to be made along with making sure the vehicle movement is smooth and non-violating traffic rules.

**Project Type:** ROS 2 / Robotics  
**Mentor:** Jaison


### Robot Platform

> _Describe the robot, its drive configuration (differential drive, quadrotor, etc.), and the autonomous task it performs._



---

## Prerequisites


- Ubuntu 22.04 (for ROS 2 Humble) or Ubuntu 24.04 (for ROS 2 Jazzy)
- [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html)
- `colcon` — `sudo apt install python3-colcon-common-extensions`
- `rosdep` — `sudo apt install python3-rosdep`
- (Optional) Gazebo Fortress / Harmonic for simulation



---

## Getting Started

```bash
git clone <your-repo-url>
cd AMCAF
```


```bash
# Source ROS 2 (add to ~/.bashrc to avoid doing this every session)
source /opt/ros/humble/setup.bash   # change to 'jazzy' if using Jazzy

# Install ROS package dependencies
sudo rosdep init        # only needed once on a fresh machine
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Launch
ros2 launch <package_name> <launch_file>.launch.py
```



---

## Project Structure

```
AMCAF/
├── src/........................ ROS 2 packages (one subdirectory per package)
├── launch/..................... Top-level launch files
├── config/..................... Parameter YAML files
├── urdf/....................... Robot description files (URDF/xacro)
├── worlds/..................... Gazebo simulation worlds
├── scripts/.................... Utility and environment setup scripts
├── docs/....................... Architecture docs, design notes
└── assets/..................... Images, diagrams, demo GIFs
```

> See [`docs/`](docs/) for detailed documentation on each component.

---

## Usage

> _Replace this section with specific usage instructions, example commands, screenshots, or a demo GIF._
>
> Example:
> ```bash
> ros2 run my_package my_node --param value
> ```

---

## Team

| Name | Role |
|------|------|
| Nishu | Intern |
| Dhruv | Intern |
| Mohit | Intern |
| Jaison | Mentor |

---

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

---

<div align="center">
  Made with ❤️ at <a href="https://www.e-yantra.org">e-Yantra, IIT Bombay</a>
</div>
