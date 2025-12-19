# ROS 2 Installation Requirements for Textbook

## Overview
This document outlines the ROS 2 installation requirements for students and researchers working with the Physical AI & Humanoid Robotics textbook. The recommended ROS 2 distribution is Humble Hawksbill, which is an LTS (Long Term Support) version with extended support until 2027.

## System Requirements

### Operating Systems
- **Ubuntu 22.04 (Jammy Jellyfish)** - Primary supported platform
- **Windows 10/11** - Secondary support with WSL2 recommended
- **macOS** - Limited support, primarily for development and simulation

### Hardware Requirements
- **Processor**: Multi-core processor (Intel i5 or equivalent recommended)
- **RAM**: Minimum 8GB (16GB recommended for simulation work)
- **Storage**: At least 10GB free space for core ROS 2 installation
- **Network**: Internet connection required for installation and package updates

## Installation Methods

### Ubuntu 22.04 - Recommended
1. Set up locale:
   ```bash
   locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. Set up sources:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. Install ROS 2 Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

4. Install Python development tools:
   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   sudo rosdep init
   rosdep update
   ```

### Windows Installation
For Windows users, we recommend using Windows Subsystem for Linux (WSL2) with Ubuntu 22.04:

1. Install WSL2 with Ubuntu 22.04
2. Follow the Ubuntu installation instructions above
3. For GUI applications, install an X11 server like VcXsrv or WSLg

### macOS Installation
macOS support is limited but possible through Docker or source compilation:
1. Use Docker with ROS 2 images
2. Or compile from source (advanced users only)

## Environment Setup

### Sourcing ROS 2
Add to your `~/.bashrc` (Linux) or `~/.zshrc` (macOS):
```bash
source /opt/ros/humble/setup.bash
```

### Python Package Installation
Install required Python packages for the textbook:
```bash
pip3 install rclpy
pip3 install ros2cli
pip3 install transforms3d  # For 3D transformations
pip3 install numpy
pip3 install matplotlib  # For visualization
```

## Verification Steps

1. Open a new terminal and verify installation:
   ```bash
   ros2 --version
   ```

2. Check available commands:
   ```bash
   ros2 --help
   ```

3. Test basic functionality:
   ```bash
   ros2 topic list
   ros2 node list
   ```

## Troubleshooting

### Common Issues
- **Package not found**: Ensure sources are properly added and updated
- **Permission errors**: Use proper sourcing and environment setup
- **Python import errors**: Ensure rclpy is properly installed in your Python environment

### Testing Python Integration
Verify Python integration with:
```python
import rclpy
print("rclpy imported successfully")
print(f"rclpy version: {rclpy.__version__ if hasattr(rclpy, '__version__') else 'unknown'}")
```

## Development Tools

### Recommended IDEs
- **VS Code** with ROS extension
- **PyCharm** with Python ROS plugins
- **Atom** with ROS packages

### Additional Tools
- **RViz2**: 3D visualization tool for ROS 2
- **rqt**: GUI tools for ROS 2
- **Gazebo**: Robot simulation environment