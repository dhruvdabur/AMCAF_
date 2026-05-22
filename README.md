<!-- TEMPLATE_NOTICE_START -->
<div align="center">

# e-Yantra Project Template

**Standardized project template for e-Yantra interns — IIT Bombay**

[![e-Yantra](https://img.shields.io/badge/e--Yantra-IIT%20Bombay-orange)](https://www.e-yantra.org)

</div>

> [!IMPORTANT]
> **Mentors — run `python setup.py` before sharing this repo with interns.**
>
> The script asks a few questions (project name, category, intern names, your name) and then automatically:
> - Creates the right folder structure for your project type (Hardware / ROS 2 / Software / FPGA)
> - Fills in and cleans up this README with your project's details
> - Generates a `.gitignore` and `LICENSE` file
> - For ROS 2 projects: scaffolds a starter package with `package.xml`, `CMakeLists.txt`, and a launch file
> - For Software/Web projects: creates a `.env.example`
>
> ```
> python setup.py
> ```
>
> After it finishes, commit everything and share the repo URL with your intern(s).
> Follow [`docs/wiki-guide.md`](docs/wiki-guide.md) to set up the project Wiki.

---
<!-- TEMPLATE_NOTICE_END -->

<div align="center">

# {{PROJECT_NAME}}

{{SHORT_DESCRIPTION}}

[![Category](https://img.shields.io/badge/Category-{{PROJECT_TYPE_BADGE}}-blue)](#)
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

{{SHORT_DESCRIPTION}}

**Project Type:** {{PROJECT_TYPE}}  
**Mentor:** {{MENTOR_NAME}}

<!-- [HARDWARE_ONLY_START] -->
### Hardware Overview

> _Describe the hardware platform, microcontroller/processor, key sensors, and actuators used in this project._
<!-- [HARDWARE_ONLY_END] -->

<!-- [ROS_ONLY_START] -->
### Robot Platform

> _Describe the robot, its drive configuration (differential drive, quadrotor, etc.), and the autonomous task it performs._
<!-- [ROS_ONLY_END] -->

<!-- [SOFTWARE_ONLY_START] -->
### System Overview

> _Describe the application, its architecture (frontend/backend/API), and the problem it solves._
<!-- [SOFTWARE_ONLY_END] -->

<!-- [FPGA_ONLY_START] -->
### FPGA / Design Overview

> _Describe the FPGA board, the HDL language used (Verilog/VHDL/SystemVerilog), and the digital system being implemented._
<!-- [FPGA_ONLY_END] -->

---

## Prerequisites

<!-- [HARDWARE_ONLY_START] -->
- A C/C++ toolchain for your target platform (e.g., `arm-none-eabi-gcc`, Arduino IDE, PlatformIO)
- Programmer or debugger (e.g., ST-Link, J-Link, USB-to-UART adapter)
- Python 3.8+ (for utility scripts and tests)
- List any specific hardware: boards, sensors, modules, power supply specs
<!-- [HARDWARE_ONLY_END] -->

<!-- [ROS_ONLY_START] -->
- Ubuntu 22.04 (for ROS 2 Humble) or Ubuntu 24.04 (for ROS 2 Jazzy)
- [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html)
- `colcon` — `sudo apt install python3-colcon-common-extensions`
- `rosdep` — `sudo apt install python3-rosdep`
- (Optional) Gazebo Fortress / Harmonic for simulation
<!-- [ROS_ONLY_END] -->

<!-- [SOFTWARE_ONLY_START] -->
- Python 3.10+ (for backend)
- Node.js 18+ and npm (for frontend, if applicable)
- Docker (optional, for containerized development)
- PostgreSQL / MongoDB / other DB as required by your project
<!-- [SOFTWARE_ONLY_END] -->

<!-- [FPGA_ONLY_START] -->
- Xilinx Vivado Design Suite (for Xilinx/AMD FPGAs) **or** Intel Quartus Prime (for Intel/Altera FPGAs)
- Simulation tool: ModelSim / Questa (commercial) **or** Icarus Verilog / Verilator (open source)
- GTKWave for viewing waveforms — `sudo apt install gtkwave`
- Python 3.8+ (for automation scripts)
- Target FPGA board (e.g., Basys 3, DE10-Nano, Arty A7)
<!-- [FPGA_ONLY_END] -->

---

## Getting Started

```bash
git clone <your-repo-url>
cd {{REPO_NAME}}
```

<!-- [HARDWARE_ONLY_START] -->
```bash
# Install toolchain dependencies — update with your specific steps
# e.g., for PlatformIO:
pip install platformio

# Build firmware
make build
# or: pio run

# Flash to target device
make flash
# or: pio run --target upload
```
<!-- [HARDWARE_ONLY_END] -->

<!-- [ROS_ONLY_START] -->
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
<!-- [ROS_ONLY_END] -->

<!-- [SOFTWARE_ONLY_START] -->
```bash
# Copy and configure environment variables
cp .env.example .env
# Edit .env with your local settings

# Backend
cd backend
pip install -r requirements.txt
python main.py

# Frontend (if applicable)
cd ../frontend
npm install
npm run dev
```
<!-- [SOFTWARE_ONLY_END] -->

<!-- [FPGA_ONLY_START] -->
```bash
# Simulate using Icarus Verilog (open source)
iverilog -o sim/out tb/<testbench>.v rtl/<module>.v
vvp sim/out
gtkwave sim/<dump>.vcd    # view waveforms in GTKWave

# Synthesis and implementation — Vivado (GUI)
# 1. Open Vivado → Create Project → add rtl/ sources and constraints/
# 2. Run Synthesis → Implementation → Generate Bitstream

# Synthesis and implementation — Vivado (Tcl, if scripts/synth.tcl is provided)
vivado -mode tcl -source scripts/synth.tcl

# Program the FPGA
# Open Vivado Hardware Manager → Connect → Program Device → select .bit file
```
<!-- [FPGA_ONLY_END] -->

---

## Project Structure

```
{{PROJECT_STRUCTURE}}
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
{{INTERN_TABLE}}
| {{MENTOR_NAME}} | Mentor |

---

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

---

<div align="center">
  Made with ❤️ at <a href="https://www.e-yantra.org">e-Yantra, IIT Bombay</a>
</div>
