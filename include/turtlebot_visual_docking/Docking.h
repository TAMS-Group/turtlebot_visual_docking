/*********************************************************************************
Copyright (c) 2017, Kolja Poreski
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
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


#pragma once 

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
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
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Vector3.h>
#include <turtlebot_visual_docking/Avg.h>
#include <turtlebot_visual_docking/DockingAction.h>

class Docking
{


private:
	struct Vector2{
                float x;
                float y;
                };
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	MoveBaseClient          *_ac;
	tf::TransformListener   *listener;
	ros::Publisher          _publisher;
	ros::Subscriber         _sub1;
	ros::Subscriber         _sub2;
	ros::Subscriber         _sub3;
	ros::Subscriber         _sub4;
	ros::Subscriber         _sub5;
	ros::Subscriber         _sub6;
	double                  _angle;
	ros::NodeHandle         *_n;
	tf::StampedTransform    _transform_cam_base;
	tf::StampedTransform    _transform_optical_cam;
	bool                    _find_tag;
	bool                    _docking_status;// Is true when robot is loading energy
	bool                    _try_more;
	bool                    _start_avg;
	bool                    _TAG_AVAILABLE;
	Avg                     *_avg_pos;
	Avg                     *_avg_dock;
	Avg                     *_avg_X;
	Avg                     *_avg_Y;
	Avg			*_avg_yaw;
	float                   _avg_position_X;
	float                   _avg_position_Y;
	float                   _avg_position_angle;
	float                   _avg_docking_angle;
	float 			_avg_yaw_angle;
	std::string             _tag_name;
	int 			_tag_id;
	bool                    _bumper_pressed;
	Vector2                 _odom_pos;
	tf::Transform           *_Odometry_Transform;
 	float 			_TURTLEBOT_PRE_DOCKING_POSE_X;
	float 			_TURTLEBOT_PRE_DOCKING_POSE_Y;	
        
	/**
	 * This function returns the absolute value of the parameter in.
 	*/
        float getAmount(float in);
	
	/**
 	 * This function calculates the angle epsilon.
 	 * Because the robot does not determine the position angle during the drive any more.
         * We can use an simple constant.
         */
	float epsi();
	
	/**
 	* This function calculates the docking angle 
 	* described in equation 4.
 	*/
	float docking_angle();
	
	/**
 	 * This function gives the angle in rad between the x-axis and the transform. 
 	 * The orign of the coordinate-system is the April-Tag. Thus the apriltag must be
 	 * visible when this function is used. 
 	 */
	float get_direction_angle();

	/**
	 * This function gets the transform between the 
 	 * camera and the base_link.
 	 * The result is stored in _trcamera_rgb_optical_frameansform_cam_base
	 */
	void get_cameraLink_baseLink();
	
	/**
	 * This function gets the transform betwwen the 
 	 * camera_rgb_optical_frame and the camera_link.
 	 * The result is stored in _transform_optical_cam
	 */
	void get_optical_frame();
	
	/**
	 * This function gets the angle alpha_yaw by using the lookupTransform
 	 * function.
 	 */
	float get_yaw_angle();

	/**
 	 * This function gets the position by using the lookupTransform function.
 	 * It returns a two dimensional Vector (Vector2).
 	 */	
 	Vector2 get_position();
	
	/**
	* This function sets some initial values for the class variables.	
	*/
	void Init();
	
	/**
	* This function registers all the CallbackFunctions used.
	*/
	void RegisterCallbackFunctions();
public:
	/**
	 * This function starts the frontal docking.
 	 * Before it does that the avarage arrays are flushed.
	 */
	void startFrontalDocking();

	/**
	* This constructor is for the use in a node, which uses the 
	* launch file: start_docking.launch
	*/
	Docking();
	
	/**
	* This constructor us used by the ActionServer
	*/
	Docking(ros::NodeHandle* nodeHandle,int tag_id);
	
	/**
	* This callback function reads the Odometry and saves 
	* it into the _odom_pos Vector
	*/
	void get_odom_pos(const nav_msgs::Odometry& msg);
	
	/**
	* This function sets the class variable _bumper_pressed to true
	* if the bumper has been trigered. Otherwise the variable _ bumper_pressed
	* should be false.
	*/
	void get_bumper_status(const kobuki_msgs::BumperEvent& msg);
	

	/**
 	* This callback function read the rotation angle, 
 	* from the IMU. The value is stored in the Varibale 
 	* _angle.
 	*/
	void actual_angle(const sensor_msgs::Imu& msg);
	
	/**
 	 * This callback function detects if the robot is in the docking 
 	 * station and is recharging. The rusult is stored in the variable 
 	 * _docking_status which is true if the robot is recharging and 
 	 * false otherwise.  
 	 */
	void get_battery_status(const diagnostic_msgs::DiagnosticArray& msg);

	/**
	 * This callback function detects if the robot is in the docking
 	 * station and does recharging.
 	 */
	void get_charging_status(const kobuki_msgs::SensorState& msg);

	/**
 	 * This callback funtion reads all angles which are used for the 
	 * docking algorithm. Because we compute an avarage over these angles
	 * we does not use TF. This function calculates the needed values out of the 
	 * 'apriltags_ros::AprilTagDetectionArray'.
	 * This function only starts working if the class variable _start_avg is 
	 * set to true.
 	 */
	void get_avg_position_angle(const apriltags_ros::AprilTagDetectionArray& msg);

	/**
 	* This callback function sets the variable _TAG_AVAILABLE to true, 
 	* when the Apriltag can be seen. Else _TAG_AVAILABLE
 	* will be set to false.
	* This function will only start working if the classvariable _find_tag
	* is set to true.
	*/	
	void findTag(const apriltags_ros::AprilTagDetectionArray& msg);
	
	/**
 	 * This function moves the robot exact about alpha_rad
 	 */
	void move_angle(float alpha_rad);
	
	/**
	 * This function moves the robot the given distance forward
	 * Distance in m.
	 */
	void drive_forward(float distance);
	/**
	 * This function moves the robot backwards about the given distance
	 * Distance in m.
	 */
	void drive_backward(float distance);
	
	/**
 	* This function dirves to the given coordinates.
 	* This function only works if the robot does know its position. 
 	*/
	void move_to(float x, float y,float A);
	
	/**
	 * The robot turns until the docking asngle is smaller than epsilon.
	 */
	void watchTag();
	
	/**
 	 * This function turns the robot counter clock-wise until the 
 	 * Apriltag can be seen by the camera.
 	 */
	void searchTag();
	
	/**
 	 * This function turns the robot in order to have an 
 	 * resulting yaw angle of zero deg.
 	 */
	void adjusting();
	
	/**
	 * This function does the linear Approach described in section 6.2
 	 */
	void linearApproach();
	/**
	 * This function should be called before any '_avg_...' variable is used
	 */
	void startReadingAngle();
	
	/**
	 * This function should be called after the reading the '_avg_...' variables
	 */
	void stopReadingAngle();
	
	/**
	 * This function does the positioning described in section 6.1
 	 */
	void positioning();	

	/**
 	* This function does the frontal docking, which is 
 	* described in section 6.3
 	*/
	void docking();
	
	/**
	 * This function starts the docking.
 	 */
	bool startDocking();	

};

