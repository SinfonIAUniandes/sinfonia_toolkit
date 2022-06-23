SINFONIA_RESOURCES
-------------
This package offers a series of ROS services that help to use the pepper controller.
**Table of content**

[TOCM]

[TOC]

###DEPENDENCES AND CONFIGURATION:
To word with sinfonia_resources you need the next packages:
- pepper_moveit_config
- pepper_robot
- pepper_virtual
- sinfonia_resources
- naoqi_driver
- naoqi_dcm_driver

To download pepper_moveit_config, pepper_robot, pepper_virtual, sinfonia_resources, naoqi_driver y naoqi_dcm_driver  :
```ROS
git clone https://github.com/ros-naoqi/pepper_moveit_config.git
git clone https://github.com/ros-naoqi/pepper_robot.git
git clone https://github.com/awesomebytes/pepper_virtual.git
git clone https://github.com/SinfonIAUniandes/sinfonia_resources.git
git clone https://github.com/ros-naoqi/naoqi_driver.git
git clone https://github.com/ros-naoqi/naoqi_dcm_driver.git
```

Before to install the packages you need install the next package to use the libraries libqiconfig and libqicoreconfig:
```ROS
rosdep install -i -y --from-paths ./naoqi_driver
```
###INSTRUCTIONS TO USE:
To use this package develop by SinfonIA members you should to create two files in your Workspace, the first should be called states.csv and the second should be called trayectories.csv.

####VIRTUAL:
To run the node that offers the manipulation services is necesary that you run Rviz with the robot model. Execute the next codes after to config your workspace:
```ROS
roslaunch sinfonia_resources demo_pepper-launch
```
In another terminal:
```ROS
rosrun sinfonia_resources Pepper_movement.py
```
At this point, you have activate the manipulation node and this is running correctly. At this moment you can to call the services, example:
```ROS
rosservice call /manipulation_utilities/goToPose TrayPose
```

####REAL ROBOT:
TO run the node in a real robot you should to change your Environment Variables at the same way as to run the [remote_controller]. To continue you should to execute the next codes:
```ROS
roslaunch pepper_dcm_bringup pepper_bringup.launch
roslaunch pepper_moveit_config moveit_planner.launch
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=192.168.1.139 network_interface:=wlp4s0
```
####SERVICES:
sinfonia_resources offers two services under manipulation_utilities:
######"manipulation_utilities/goToPose":
This service allows change the robot position. This service use the group "both_arms_hands" and
has three different poses ("TrayPose-SackPose-Standard")

######"manipulation_utilities/executeTrayectory":
This service allows execute trayectories whit the end to dance or execute social movements. This service use the group "both_arms_hands" and  has three different poses ("Wave-Macarena")

######"manipulation_utilities/savepose":
This service allows save poses to make new movements or create a new pose. This service use the group "both_arms_hands". To use it you should to call the service and enter a pose name. The services save the joint coordinate information in the states.csv and you can to access to this pose with the serve goToPose.

######"manipulation_utilities/openhand":
This service allows open or close the robot's hand. If you call the services and enter the bool "False" the hand is closed. If you call the services and enter the bool "True" the hand is open.
