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

#include "std_msgs/String.h"
#include <iostream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <docking/DockingAction.h>
#include <Docking.h>
#include <thread>
#include <future>

class DockingServer {

protected:

  ros::NodeHandle _nh;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs. 
  actionlib::SimpleActionServer<docking::DockingAction> _as; 
  std::string _action_name;

  //create messages that are used to published feedback/result
  docking::DockingFeedback _feedback;
  docking::DockingResult   _result;


 // Parameters from the start_docking.launch file
 int	_tag_id;
 std::future<bool> _future_result;
 std::future<void> _future_detection;
private:

   void startApriltagDetectionNode(){

	// At first we have to start the TagDetection Node for the Apriltag detection
        // we have to run this in an extrqa thread

        //std::string tagid_str = std::to_string(_tag_id);
        //std::string cmd = "roslaunch docking start_tagdetection.launch tag_param:='[{id: "+tagid_str+",size: 0.120}]'";
        //system(cmd.c_str());
        ros::Duration(2.0).sleep();


   }  



   bool runDocking(){
	ROS_INFO("runDocking()"); 
	_feedback.text.clear();
	Docking *d = new Docking(&_nh,_tag_id,&_feedback);
	return d->startDocking();
   }



  void runDockingThread(){

	//_future_detection = std::async(std::launch::async,&DockingServer::startApriltagDetectionNode,this);
	//ros::Duration(3.0).sleep();
	
	_future_result = std::async(std::launch::async,&DockingServer::runDocking,this);

   }



public: 

// Constructor 
DockingServer(std::string name) :
    _as(_nh, name, boost::bind(&DockingServer::executeCB, this, _1), false),
    _action_name(name)
  {
    _as.start();
  }

// Destructor
  ~DockingServer(void)
  {
  }


void executeCB(const docking::DockingGoalConstPtr &goal){

// Read the tagid and compare it to the default tagid defined in /etc/environment 
_tag_id  = goal->tagid;

// We start the docking algorithm in a different thread,
// because we want to be able to listen to an cancel message
// while the docking algortithm is running.
// TODO: Docking algorithm sould produce feedback-messages
runDockingThread();

// After running the docking algorithm we wait until the algorithm has 
// finished or an cancel massage has been send.  
std::future_status status;
do{
 
	//int soa = ARRAY_SIZE(_feedback.text);
	//ROS_INFO("%i",soa); 
	//_as.publishFeedback(_feedback);

	status = _future_result.wait_for(std::chrono::seconds(2));
     if (_as.isPreemptRequested() || !ros::ok()){
        	ROS_INFO("Cancel...");
        	// set the action state to preempted
        	_as.setPreempted();
        	// Now we have to kill the DockingThread
		exit(0);
    }
}while(status != std::future_status::ready);
bool success = _future_result.get();
if(success){
	ROS_INFO("reached");
	_result.text = "Arrived on docking station successfully.";
	_as.setSucceeded(_result);
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
