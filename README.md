# turtlebot_visual_docking
This repository provides the algorithm which is needed to do the docking with
the Apriltags as landmark. All required system variables like the HOME_POSE etc. need to be in 
the file: /etc/environment  on the turtlebots file system. This file should contain the following Variables.
E.g.:
export TURTLEBOT_NAME="mikey"
export TURTLEBOT_BASE="kobuki"
export TURTLEBOT_LASER="hokuyo_04lx"
export TURTLEBOT_3D_SENSOR="kinect"
export TURTLEBOT_STACKS="hexagons"
export TURTLEBOT_HOME_POSE_X="2.97"
export TURTLEBOT_HOME_POSE_Y="-0.46"
export TURTLEBOT_HOME_POSE_A="-1.56"
export TURTLEBOT_PRE_DOCKING_POSE_X="2.67"
export TURTLEBOT_PRE_DOCKING_POSE_Y="0.606"
export TURTLEBOT_PRE_DOCKING_POSE_A="-1.56"
export RPC_TAG_ID="3"
export RPC_DOCK_ID="99"


__Usage bringup__

To bring up the turtlebot including the mobile base, the kobuki auto docking and the sensors you can use:

``` roslaunch turtlebot_visual_docking justbringup.launch ```

__Usage ActionServer__

To start the ActionServer you can use:

``` roslaunch turtlebot_visual_docking docking_server.launch ```

To start docking without driving to the HOME_POSE use:
``` rostopic pub /DockingActionServer/goal ... ```


To start docking with driving to the HOME_POSE use:
``` rostopic pub /GoHomeActionServer/goal ... ```


__Usage docking__

To start the docking algorithm you can use:

``` roslaunch turtlebot_visual_docking start_docking.launch ```
