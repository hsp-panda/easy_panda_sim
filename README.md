# easy_panda_sim

![CI badge](https://github.com/hsp-panda/easy_panda_sim/workflows/ROS%20CI%20Workflow/badge.svg)

A lightweight ROS package for simulation with Franka Panda Emika on Gazebo.

It has been tested on Ubuntu 18.04 with ROS Melodic.

### Supported features

- ROS controllers:
  - [State publishing](https://github.com/hsp-panda/easy_panda_sim/blob/f7571525ecef6925dda269b56155ef1274a35b61/config/panda_control.yaml#L1) (`joint_state_controller`)
  - [Joint control](https://github.com/hsp-panda/easy_panda_sim/blob/f7571525ecef6925dda269b56155ef1274a35b61/config/panda_control.yaml#L5) (`position_joint_trajectory_controller`)
- Support for Franka gripper actions `/franka_gripper/{homing, move, grasp}`
  - `grasp` and `move` support the parameters `width` and `speed`
- Support for [hsp-panda/panda_grasp_server](https://github.com/hsp-panda/panda_grasp_server)


### Missing features

- Support for parameters `force` , `epsilon.{inner, outer}` in action `/franka_gripper/grasp`
- Support for RGB-D vision
- Cartesian space control using ROS controllers
- Python, C++ basic examples


### How to install

> We assume that you have a working ROS workspace `<catkin_ws>`
```
cd <catkin_ws>/src
git clone https://github.com/hsp-panda/easy_panda_sim
cd <catkin_ws>
catkin build
source <catkin_ws>/devel/setup.bash
```

### How to run

```
roslaunch easy_panda_sim simulation.launch
```

### Test joint space control via GUI

> We assume that you have installed `ros-melodic-rqt-joint-trajectory-controller` via apt

```
roslaunch easy_panda_sim simulation.launch
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover
```

<p align="center"><img src="https://github.com/hsp-panda/easy_panda_sim/blob/master/assets/gazebo_rqt.png"/></p>

### Maintainers
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/xenvre.png" width="40">](https://github.com/xenvre) | [@xenvre](https://github.com/xenvre) |
