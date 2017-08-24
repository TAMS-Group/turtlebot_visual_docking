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

//#pragma once

#include "std_msgs/String.h"
#include <iostream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_visual_docking/DockingAction.h>
#include <turtlebot_visual_docking/GoHomeAction.h>
#include <turtlebot_visual_docking/Docking.h>

class DockingServer {

protected:

  ros::NodeHandle _nh;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs. 
  actionlib::SimpleActionServer<turtlebot_visual_docking::DockingAction> _as_tagid;
  actionlib::SimpleActionServer<turtlebot_visual_docking::GoHomeAction>  _as_gohome; 
  std::string _action_name;

  //create messages that are used to published feedback/result
  turtlebot_visual_docking::DockingFeedback _feedback;
  turtlebot_visual_docking::DockingResult   _result;

  turtlebot_visual_docking::GoHomeFeedback  _gh_feedback;
  turtlebot_visual_docking::GoHomeResult    _gh_result;	

 // Parameters from the start_docking.launch file
 int	_tag_id;
 double _HOME_POSE_X;
 double _HOME_POSE_Y;
 double _HOME_POSE_A;

private:

   bool runDocking(){
	ROS_INFO("runing Docking ..."); 
	Docking *d = new Docking(&_nh,_tag_id);
	return d->startDocking();
   }

  bool runGoHome(){
        ROS_INFO("running GoHome ... ");
        Docking *d = new Docking(&_nh,_tag_id);
	// First we need to calculate the PRE_DOCKING_POSE
	// The robot should stand 60cm in front of the docking station
	// and 30cm left of the docking station
	float X     = _HOME_POSE_X - 0.60;
	float Y     = _HOME_POSE_Y - 0.30;
	float phi   = _HOME_POSE_A;
	float Y_map = X*cos(phi) + Y*sin(phi);
	float X_map = -X*sin(phi) + Y*cos(phi);
	
	ROS_INFO("PreDockingPose: ( % f , %f )",X_map,Y_map);


	d->move_to(X_map,Y_map,_HOME_POSE_A);
        return d->startDocking();
   }

public: 

// Constructor 
DockingServer(std::string name) :
    _as_tagid(_nh, name, boost::bind(&DockingServer::executeDocking, this, _1), false),
    _action_name(name),
    _as_gohome(_nh,"GoHomeActionServer",boost::bind(&DockingServer::executeGoHome, this, _1),false)
  {

    _as_tagid.registerPreemptCallback(boost::bind(&DockingServer::cancel_docking, this));
    _as_gohome.registerPreemptCallback(boost::bind(&DockingServer::cancel_gohome, this));	
    _as_tagid.start();
    _as_gohome.start();	  
}

// Destructor
  ~DockingServer(void)
  {
  }


void cancel_docking(){
	ROS_INFO("Cancel");
        _as_tagid.setPreempted();
}

void cancel_gohome(){
	ROS_INFO("Cancel");
	_as_gohome.setPreempted();
}

/**
 * This is the callback function if the robot receives an GoHome Message.
 * The robot should drive to the PRE_DOCKING_POSE by using moveBase. 
 * Therefore the robot must know where its location is.  
 */
void executeGoHome(const turtlebot_visual_docking::GoHomeGoalConstPtr &home){


ROS_INFO("GO HOME");


double HOME_POSE_X = 2.97;
double HOME_POSE_Y = -0.46;
double HOME_POSE_A = -1.56;
int    ID 	   = 99;
if (_nh.getParam("/dock_server/HOME_POSE_X", HOME_POSE_X)){
	_HOME_POSE_X = HOME_POSE_X;
}
if (_nh.getParam("/dock_server/HOME_POSE_Y", HOME_POSE_Y)){
	_HOME_POSE_Y = HOME_POSE_Y;
}
if (_nh.getParam("/dock_server/HOME_POSE_A", HOME_POSE_A)){
        _HOME_POSE_A = HOME_POSE_A;
}
if (_nh.getParam("/dock_server/TAG_ID", ID)){
        _tag_id = ID;
}

	ROS_INFO(" HOME POSE: (%f , %f)",_HOME_POSE_X,_HOME_POSE_Y);
	bool success = runGoHome();
	if(success){
        	ROS_INFO("reached");
        	_result.result = "Successfully";
		_gh_result.result = "Successfull";
        	_as_tagid.setSucceeded(_result);
		_as_gohome.setSucceeded(_gh_result);
		}	
}



void executeDocking(const turtlebot_visual_docking::DockingGoalConstPtr &goal){

// Read the tagid and compare it to the default tagid defined in /etc/environment 
_tag_id  = goal->tagid;

bool success = runDocking();

if(success){
	ROS_INFO("reached");
	_result.result= "Successfull.";
	_as_tagid.setSucceeded(_result);
}
}


};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "DockingActionServer");

  DockingServer docking("DockingActionServer");
  ros::spin();

  return 0;
}
