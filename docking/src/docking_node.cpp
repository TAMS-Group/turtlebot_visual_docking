
#include <Docking.h>

int main(int argc, char **argv){

        ROS_INFO("Docking is started!");
	ROS_INFO("Hopefully the robot knows where he is!");
        ros::init(argc,argv,"follow_tag");
        ros::AsyncSpinner spinner(5);
        spinner.start();
        Docking *d = new Docking();
       
	d->startDocking();	
	//d->drive_forward(0.15);
	//ros::spin();
        return 0;
}
