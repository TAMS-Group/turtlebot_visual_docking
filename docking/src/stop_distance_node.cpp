/*********************************************************************************
Copyright (c) 2017, Kolja Poreski
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the TAMS and HANDARBEITS-HAUS Poreski KG nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL KOLJA PORESKI BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include  <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float64.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <iostream>
#include <math.h> 
#include <time.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h> 
#include <tf/LinearMath/Vector3.h>
#include "avg.cpp"
#include <mutex>

class SD {

private:
tf::TransformListener   *listener;
ros::NodeHandle		_n;
std::string		_tag_name;
ros::Subscriber         _sub;
public:

SD(){
	listener = new tf::TransformListener();
	
	std::string tag_id;
        if (_n.getParam("/dock/tag_id", tag_id)){
                _tag_name = "tag_" + tag_id;
        }else{
                ROS_ERROR("No Parameters found!");
                exit(0);
        }
	
	_sub = _n.subscribe("tag_detections",1,&SD::callback,this);
}

void callback(const apriltags_ros::AprilTagDetectionArray& msg){


	get_position();
}


void get_position(){


        tf::StampedTransform transform;

        try{
           ros::Time begin = ros::Time::now();

           listener->waitForTransform(_tag_name,"base_link",begin,ros::Duration(5.0));
           listener->lookupTransform(_tag_name, "base_link",begin, transform);

        }catch (tf::TransformException ex){
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
        }

        float x =  transform.getOrigin().x();
        float y =  transform.getOrigin().z();

        ROS_ERROR("Distance: %f",y);


}
};


int main(int argc, char **argv){

        ros::init(argc,argv,"StopDistance");
	
        ros::AsyncSpinner spinner(5);
	spinner.start();


	SD *stopD = new SD();

	stopD->get_position();
	ros::spin();

        return 0;

}

