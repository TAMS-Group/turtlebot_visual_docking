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
 std::string	_tag_id;

private:


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
int msg_tag_id 	  = goal->tagid;
ROS_INFO("tag_id = %i",msg_tag_id);
std::string tag_id = "tag_" +  std::to_string(msg_tag_id);

Docking *d = new Docking(&_nh,tag_id);

d->startDocking();




_result.text = "Arrived on docking station successfully.";
_as.setSucceeded(_result);
}


};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "DockingActionServer");

  DockingServer docking("DockingActionServer");
  ros::spin();

  return 0;
}
