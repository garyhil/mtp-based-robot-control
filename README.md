
# mtp-based-robot-control

Welcome to the MBRC-Repository, a project aimed at facilitating the control of robots through the Module Type Package Standard. This Repository is based on the Publication: [MTP based Integration of Robots 
in Modular Production Environments](https://www.researchgate.net/profile/Gary-Hildebrandt)

## Module Type Package (MTP)
The Module Type Package is an industry standard in the process industry, enabling a standardized description of modules. The MTP itself encompasses aspects such as data objects, services, interfaces, communication, alarms, and operating screens of the modules. It is a hierarchically organized and containerized information model, created during module engineering and provided for exchange as an AutomationML-Object-Model (.aml file). An MTP-capable orchestration can read this file, obtain all necessary infor-mation for the automated integration and control of the module within the modular overall plant, and execute its functionalities according to the product recipes.

If you want to learn more about the MTP take a look at the standard:
[VDI/VDE/NAMUR 2658 Blatt 1:2022-01 - Draft](https://www.beuth.de/en/draft-technical-rule/vdi-vde-namur-2658-blatt-1/347143763)


## Getting Started
To get started with the Modelica examples, follow these steps (Note that everything presented here was tested on a Ubuntu 20.04 LTS Machine)


## Install a ROS:
ROS, or Robot Operating System, is an open-source middleware framework widely used in robotics for developing and controlling robotic systems. It provides a flexible and modular architecture, enabling seamless communication between different components and facilitating the development of complex robotic applications.Although ROS2 is already available, there are no fitting packages released from Franka Robotics yet. Therefore we stick to the ROS1-Noetic implementation. However, transferring the concepts to ROS2 shouldn't be a problem.

To install ROS, follow the instructions [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Install the respective MoveIt Package

MoveIt is an open-source software framework designed for motion planning, manipulation, and control of robotic systems. Developed for the ROS (Robot Operating System) ecosystem, MoveIt simplifies the implementation of complex robotic tasks, offering a comprehensive set of tools for motion planning, kinematics, collision checking, and grasping. It is widely utilized in various robotic applications, empowering developers to efficiently program and control the movement of robot arms in diverse environments.

To install ROS, follow the instructions [here](https://moveit.ros.org/install/)

### Check installations
If everything worked fine, you should be able to start a virtual representation of the Franka Emika Panda Robot via the command:
```roslaunch panda_moveit_config demo.launch```

## Whats inside the repository?
1. **mtppy**
In this folder you find the respective files of the mtppy-package. This was developed from the TU Dresden P2O Lab and the original package can be found [here](https://github.com/p2o-lab/MTPPy/tree/master).
The publication to the package is [here](https://dl.acm.org/doi/abs/10.1109/ETFA52439.2022.9921713)


2. **classes**
This directory contains the ```frankaCtrlClass.py```. It defines the FrankaControl Base Class as well as the respective Move and Hand Class, which control the robots axis and end-effector.

3. **service**
In here we have the ```robotService.py```. It contains the definitions of the two MTP Services with their respective procedures.

4. **robot_positions**
This is just a simple python-File with pre defined Roboter-Poses, which are used as default values for the Services and for the control via the Orchestration.

5. **MTP-Files**
In here are the template XML as described in the [mtppy GitHub Repo](https://github.com/p2o-lab/MTPPy/tree/master) as well as one generated MTP-File for the Robot with its two Services.

6. **MTP_Robot_Services.py**
This File configrues the Robot as a Module, imports the Services and starts an OPC UA Server. With it the robots functions become available to a MTP-based Orchestration.

7. **robot_ctrl.py**
This Script sets up an OPC UA Client and emulates a mtp-based Orchestration. Via respective manipulation of the OPC UA Nodes, the Robot performs a pick&place Task based on the Move- and Hand-Service.


## Demo
To start the demonstration (we will use the virtual robot) follow theese steps:

1. run ```roslaunch panda_moveit_config demo.launch``` in your terminal. This should also start the RViz Plug-In and you should see a virtual Franka Emika Panda
2. in a second terminal run ```python3 <directory_to_File>/MTP_Robot_Services.py```. This should start the OPC UA Server at the Endpoints specified in the script.
3. in a third terminal run ```python3 <directory_to_File>/robot_ctrl.py```. This will start the client and you can run through the pick&place task by pressing enter.

You can also control a real Franka-Emika-Panda Robot. However, therefore you need to
1. Set up a real-time capable Version of Ubuntu as described [here](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
2. Install the [Libfranka](https://frankaemika.github.io/docs/installation_linux.html)]and [FrankaROS](https://frankaemika.github.io/docs/installation_linux.html) 
3. Connect the Franka Emika to the Computer and activate the [FCI Mode](https://frankaemika.github.io/docs/getting_started.html)
4. run ```roslaunch franka_example_controllers joint_impedance_example_controller.launch robot_ip:=<fci-ip> load_gripper:=<true|false> robot:=<panda|fr3>``` in your terminal. This should also start the RViz Plug-In and you should see a virtual Franka Emika Panda which mimics the behaviour of your real robot.
5. Go to point 2. in the step-instructions above.


## Authors

- [@garyhil](https://www.github.com/garyhil)


## Contributing

Contributions are always welcome!


## Badges
[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)
