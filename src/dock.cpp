
#include <turtlebot_visual_docking/Docking.h>

int main(int argc, char **argv){

        ROS_INFO("Docking is started!");
        ros::init(argc,argv,"follow_tag");
        ros::AsyncSpinner spinner(5);
        spinner.start();
        Docking *d = new Docking();
       
	d->startDocking();
	//ros::spin();
        return 0;
}
