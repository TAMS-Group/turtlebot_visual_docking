# turtlebot_visual_docking
This repository provides the algorithm which is needed to do the docking with
the Apriltags as landmark. All required system variables like the HOME_POSE etc. need to be in 
the file: /etc/environment  on the turtlebots file system. This file should contain the following Variables.
E.g.:  
export TURTLEBOT_HOME_POSE_X="2.97"  
export TURTLEBOT_HOME_POSE_Y="-0.46"  
export TURTLEBOT_HOME_POSE_A="-1.56"   
export DOCK_ID="99"  


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
