# Intralogistic-robot-simulation Installation Guide

This document provides the commands and file structure to install and run the `logistic` ROS 2 package and working of the `DummySensor` plugin for Gazebo Harmonic.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim8)

## File Structure
```
logistic/
├── src/
│   └── DummySensor.cc
├── include/
│   └── logistic/
│       └── DummySensor.hh
├── config/
│   └── gz_bridge.yaml
├── launch/
│   └── world_launch.py
├── worlds/
│   └── model.sdf
├── LICENSE
├── CMakeLists.txt
└── package.xml
```

## Prerequisites 
- Should have ROS2 Jazzy installed. You can refer [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for the setup. Follow the exact same steps.
- Source install gazebo Harmonic. This way if you encounter any issue you can just delete the workspace and do clean install again. You can refer [here](https://gazebosim.org/docs/harmonic/install_ubuntu_src/) for the setup.



## Installation

```bash

# Create a ROS 2 Workspace
mkdir -p ~/work_ws/src
cd ~/work_ws
colcon build

# Clone the Project package
cd ~/work_ws/src
git clone https://github.com/ChinmayMundane/Intralogistic-robot-simulation.git


# Build the Project
cd ~/work_ws
colcon build
```


## Run Simulation

```bash
#Task1 : simulation setup

# Source the Workspace
source ~/work_ws/install/setup.bash

# Export plugin path
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/work_ws/install/logistic/lib/logistic

# Run the Simulation
ros2 launch logistic world_launch.py

```     


## Execute DummySensor Plugin

```bash
#Task2 : creating DummySensor Plugin

# After launching the world, open another terminal
ros2 topic echo /dummy_sensor_output
```
</br > 


## Result
  
![img4](https://github.com/user-attachments/assets/a156856d-e50b-48a7-9016-60f97bbe0517)


</br > 


(Note - once you start the simulation, echo the sensor output topic on another terminal and then start/resume the simulator )











