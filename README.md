# easy_panda_sim

![CI badge](https://github.com/hsp-panda/easy_panda_sim/workflows/ROS%20CI%20Workflow/badge.svg)

A lightweight ROS package for simulation with Franka Panda Emika on Gazebo.

It has been tested on Ubuntu 18.04 with ROS Melodic.

### Disclaimer

This content of this repository takes insipiration from [cpezzato/panda_simulation](https://github.com/cpezzato/panda_simulation). Original license files of the aforementioned repository are available in this repository as [panda_simulation_LICENSE_0](panda_simulation_LICENSE_0) and [panda_simulation_LICENSE_1](panda_simulation_LICENSE_1).

### Supported features

- Joint control using ROS controllers

### Missing features

- Cartesian space control using ROS controllers
- Python, C++ basic examples
- Support for [hsp-panda/panda_grasp_server](https://github.com/hsp-panda/panda_grasp_server)


### How to run
> We assume that you have a working ROS workspace `<catkin_ws>`
```
cd <catkin_ws>/src
git clone https://github.com/hsp-panda/easy_panda_sim
cd <catkin_ws>
catkin build
source <catkin_ws>/devel/setup.bash
roslaunch easy_panda_sim simulation.launch
```
