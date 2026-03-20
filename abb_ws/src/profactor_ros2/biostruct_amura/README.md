Luca Grigolin at PROFACTOR GmbH 

#   [BIOSTRUCT AMURA]

This package contains the ros-based Planner and Executor nodes of the Biostruct Draping Software.
This is the final version with the ProPoint gripper, the Amura draping cell and the vision camera
mounted in the ProPoint gripper.

![Nodes Map](visual_images/nodes_MAP_ROS2.png)

## [PLANNER]

The activity of the Planner node is assumed to be supervised by the coordinator node, 
it has not uniquely path-planning functions but alis responsible for the whole 
ABB IRB-6700_205_280 Robot integration in the software. Its main functions are to

- Initialize the system 
- Extrinsic Calibration mode
- Saving a Placing Pose or vision pose to memory
- Path Planning in the draping routine

## [EXECUTOR]

The activity of the Executor node just need a trajectory message to be published in the topic
coordinator/trajectories and a trigger message published in the coordinator/execution topic to
execute the latest published trajectory. It's activity is coordinated with the planner by the 
coordinator.

## [MEMORY]

The memory containing the joint-value poses for the robot routine are at this location:

	src/profactor_ros2/biostruct_amura/memory/memory.yaml

## Launch the whole software 

    cd /BioStruct_Drapebot/coordinator_ws/coordinator_app/launch 

    - python3 launch_stack.py --robot-ip 192.168.125.1 
        (Real Hardware = 192.168.125.1 or Virtual Hardware = 192.168.125.20)

    - python3 launch_stack.py (Fake Hardware)


## Build the Ros Packages

	- colcon build --symlink-install

## [NOTE!]

-	The ProPoint collision gripper need to be properly configured here:

		profactor_ros2/abb_irb6700_205_280/abb_irb6700_205_280_support/urdf/abb_irb6700_205_280.xacro

-   The tool_camera frame needs to be defined as it will be mounted on the gripper, no precision is 
	needed since this is just needed for the calibration phase to make the camera always look at the 
	calibration board, the frame can be defined here:

		profactor_ros2/abb_irb6700_205_280/abb_irb6700_205_280_support/urdf/abb_irb6700_205_280_macro.xacro

-   The gripper touches the link_6 so disable collisions between this two must be set here:

		profactor_ros2/abb_irb6700_205_280/abb_irb6700_205_280_moveit_config/config/abb_irb6700_205_280.srdf


## [Local Info]

- Ubuntu 24.04.3 LTS
- ROS2 jazzy
- NVIDIA-SMI 580.95.05 
- Driver Version: 580.95.05 
- CUDA Version: 13.0  
- MQTT broker: mosquitto version 2.0.22


## Instructions to run to launch just the two nodes:

TERMINAL (1): 

	ros2 launch abb_workcell_bringup abb_control.launch.py use_fake_hardware:=true

TERMINAL (2): 

	ros2 launch biostruct_robotics_lab move_group.launch.py

TERMINAL (3):  

	ros2 launch biostruct_robotics_lab planning_node.launch.py

TERMINAL (4): 

	ros2 launch biostruct_robotics_lab execution_node.launch.py


## Launch on the real robot

launch RWS node:

	ros2 launch abb_workcell_bringup abb_rws_client.launch.py robot_ip:=192.168.125.1

launch ABB control node:

	ros2 launch abb_workcell_bringup abb_control.launch.py rws_ip:=192.168.125.1

trigger EGM:

	ros2 service call /rws_client/start_egm_joint abb_robot_msgs/srv/TriggerWithResultCode

launch Software:

	cd coordinator_ws/coordinator_app/launch
	./launch_biostruct_draping_software.py 


## Launch on virtual robot (RobotStudio)

launch RWS node:

	ros2 launch abb_workcell_bringup abb_rws_client.launch.py 

launch ABB control node:

	ros2 launch abb_workcell_bringup abb_control.launch.py 

trigger EGM:

	ros2 service call /rws_client/start_egm_joint abb_robot_msgs/srv/TriggerWithResultCode

launch Software:

	cd coordinator_ws/coordinator_app/launch
	./launch_biostruct_draping_software.py 



# EGM - RWS - ABB general control Commands 



For basic control out of ROS

## IRC5 controller - State Machine

- Flex Pendant (and/or Operator Window) should show periodically a message: "Idling..."

- depending on current operating mode, running the stat machine may require

	- holding the dead man switch

	- holding the play button
 
## ROS - RWS Client

- interface to communicate with the IRC5 controller over RWS

- provides various ros2 services to trigger RWS commands

	- e.g.: /rws_client/start_egm_joint

- depending on the current IRC5 operating mode some commands may not work

	- e.g. PP_TO_MAIN

	- [ ] #TODO research if this is documented behavior?

		- is this due to RWS limitations?

		- is this due to RWS Client limitations?

### Launch

e.g.:

```

ros2 launch abb_workcell_bringup abb_rws_client.launch.py rws_ip:=172.20.100.1

```
 
## ROS - ABB Controller

- provides a ros2_control node

	- is this the correct term? or is it device?

- with use_fake_hardware == false

	- ros2_control is based on the egm connection to the real robot

	- requires that the IRC5 controller initiates the EGM connection with the ROS node

		- is a direct function of the state machine

			- can be enabled by using a service provided by the RWS client

- with use_fake_hardware == true

	- ros2_control is based on a virtual robot

### Launch

```

ros2 launch abb_workcell_bringup abb_control.launch.py rws_ip:=172.20.100.1

```

### Start EGM communication with RWS Client

(or use rws_calls/start_egm bash script from repository)

```

ros2 service call /rws_client/start_egm_joint abb_robot_msgs/srv/TriggerWithResultCode

```
 
## ROS - motion planner & executor

- any motion planner & executor example that works with abb_workcell_bringup / abb_control.launch.py works

### Launch

e.g.: interactive moveit

```

ros2 launch abb_workcell_bringup abb_moveit.launch.py

```
 
# Notes

- quite a strong overshoot can be observed

- with abb_moveit.launch.py planning consecutive trajectories sometimes often fails, with start position in plan/reality not within tolerance

## ATTENTION

- robot moves to 0,0,0 position on egm start

	- this should have been fixed in abb_ros2?!
 

