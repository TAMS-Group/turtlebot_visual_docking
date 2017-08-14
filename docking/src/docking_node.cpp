
#include <Docking.h>

int main(int argc, char **argv){

        ROS_INFO("Docking is started!");
        ros::init(argc,argv,"follow_tag");
        ros::AsyncSpinner spinner(5);
        spinner.start();
        Docking *d = new Docking();
       
//	d->startDocking();
//	d->drive_forward(0.5);
	d->linearApproach();
	//ros::spin();
        return 0;
}
