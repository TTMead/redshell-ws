# Redshell ROS Workspace
This repo contains the ROS2 Humble workspace for the redshell droid.

## Architecture
<p align="center">  
  <img src="documentation/architecture.drawio.png">  
</p>  


## Setup Instructions
- Install Ubuntu 22 Desktop
- Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Clone the `redshell_ws` repository along with its submodules
- Source ROS2 and from the `redshell_ws/` directory run `colcon build` to compile the packages

## Run
The redshell launch files can be found in `src/redshell_bringup/launch/`. To bringup one of these configurations run,
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch redshell_bringup [launch file]
```