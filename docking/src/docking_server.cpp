#include "std_msgs/String.h"
#include <iostream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <docking/DockingAction.h>
//#include "docking_node.cpp"



class DockingServer {

protected:

  ros::NodeHandle _nh;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs. 
  actionlib::SimpleActionServer<docking::DockingAction> _as; 
  std::string _action_name;

  //create messages that are used to published feedback/result
  docking::DockingFeedback _feedback;
  docking::DockingResult   _result;


private:

std::string read_tagid(int msg_tag_id){

	std::string       param_tag_id;
	std::string       result_tag_id;
	if (_nh.getParam("/dock/tag_id", param_tag_id)){
        	   result_tag_id = "tag_" + param_tag_id;
           	   ROS_INFO(" tag_id = %s",result_tag_id.c_str());
	}
 
	if(msg_tag_id != -1){ // If there is no tag id in the message 
        	std::ostringstream ss;
		ss << msg_tag_id;
		std::string res(ss.str());
		result_tag_id = "tag_" + res;
	}

	return result_tag_id; 

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
int msg_tag_id 	  = goal->tagid;
//Get the default tagid
std::string tag_id     = read_tagid(msg_tag_id); 

ROS_INFO("tag_id = %s",tag_id.c_str());

//Docking *d = new Docking();
//d->startDocking();


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
