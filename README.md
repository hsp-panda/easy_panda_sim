# easy_panda_sim

![CI badge](https://github.com/hsp-panda/easy_panda_sim/workflows/ROS%20CI%20Workflow/badge.svg)

A lightweight ROS package for simulation with Franka Panda Emika on Gazebo.

It has been tested on Ubuntu 18.04 with ROS Melodic.

### Disclaimer

This content of this repository takes insipiration from [cpezzato/panda_simulation](https://github.com/cpezzato/panda_simulation). Original license files of the aforementioned repository are available in this repository as [panda_simulation_LICENSE_0](panda_simulation_LICENSE_0) and [panda_simulation_LICENSE_1](panda_simulation_LICENSE_1).

### Supported features

- ROS controllers:
  - Basic [state publishing](https://github.com/hsp-panda/easy_panda_sim/blob/f7571525ecef6925dda269b56155ef1274a35b61/config/panda_control.yaml#L1) (`joint_state_controller`)
  - [Joint control](https://github.com/hsp-panda/easy_panda_sim/blob/f7571525ecef6925dda269b56155ef1274a35b61/config/panda_control.yaml#L5) (`position_joint_trajectory_controller`)

### Missing features

- Cartesian space control using ROS controllers
- Python, C++ basic examples
- Support for [hsp-panda/panda_grasp_server](https://github.com/hsp-panda/panda_grasp_server)


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


