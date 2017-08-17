# turtlebot_visual_docking
This repository provides the algorithm which is needed to do the docking with
the Apriltags as landmark.


__Usage bringup__

To bring up the turtlebot including the mobile base, the kobuki auto docking and the sensors you can use:

``` roslaunch turtlebot_visual_docking justbringup.launch ```

__Usage ActionServer__

To start the ActionServer you can use:

``` roslaunch turtlebot_visual_docking docking_server.launch ```

__Usage docking__

To start the docking algorithm you can use:

``` roslaunch turtlebot_visual_docking start_docking.launch ```
