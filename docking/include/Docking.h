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


#ifndef DOCKING_H_
#define DOCKING_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
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
#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Vector3.h>
#include <Avg.h>
#include <mutex>
#include <docking/DockingAction.h>

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
	ros::Subscriber         _sub7;
	ros::Subscriber         _sub8;
	double                  _angle;
	ros::NodeHandle         *_n;
	docking::DockingFeedback *_feedback;
	tf::StampedTransform    _transform_cam_base;
	tf::StampedTransform    _transform_optical_cam;
	int                     _ticks_right;
	int                     _ticks_left;
	bool                    _find_tag;
	float                   _start_pos_x;
	float                   _start_pos_y;
	float                   _start_pos_z;
	bool                    _docking_status;// Is true when robot is loading energy
	bool                    _try_more;
	bool                    _start_avg;
	bool                    _TAG_AVAILABLE;
	Avg                     *_avg_pos;
	Avg                     *_avg_dock;
	Avg                     *_avg_X;
	Avg                     *_avg_Y;
	std::mutex              _g_mutex;
	float                   _avg_position_X;
	float                   _avg_position_Y;
	float                   _avg_position_angle;
	float                   _avg_docking_angle;
	std::string             _tag_name;
	int 			_tag_id;
	bool                    _bumper_pressed;
	Vector2                 _odom_pos;
	tf::Transform           *_Odometry_Transform;
 	float 			_TURTLEBOT_PRE_DOCKING_POSE_X;
	float 			_TURTLEBOT_PRE_DOCKING_POSE_Y;	
        float getAmount(float in);
	Vector2 normalize(Vector2 vec);
	Vector2 spinVector(Vector2 vec,float angle);
	float epsi(float _tx);
	float docking_angle();
	float get_direction_angle();
	void get_cameraLink_baseLink();
	void get_optical_frame();
	void Show_Quad(tf::Quaternion q);
	float get_yaw_angle();
	Vector2 get_position();
	void Init();
	void RegisterCallbackFunctions();
public:

	void startFrontalDocking();
	Docking();
	Docking(ros::NodeHandle* nodeHandle,int tag_id,docking::DockingFeedback* feedback);
	void get_odom_pos(const nav_msgs::Odometry& msg);
	void get_ticks(const kobuki_msgs::SensorState& msg);
	void get_bumper_status(const kobuki_msgs::BumperEvent& msg);
	void actual_angle(const sensor_msgs::Imu& msg);
	void get_battery_status(const diagnostic_msgs::DiagnosticArray& msg);
	void get_charging_status(const kobuki_msgs::SensorState& msg);
	void get_avg_position_angle(const apriltags_ros::AprilTagDetectionArray& msg);
	void findTag(const apriltags_ros::AprilTagDetectionArray& msg);
	void move_angle(float alpha_rad);
	void drive_forward(float distance);
	void drive_backward(float distance);
	void move_to(float x, float y);
	void RememberPosition();
	void watchTag();
	void searchTag();
	void adjusting();
	void linearApproach();
	void startReadingAngle();
	void stopReadingAngle();
	void positioning();	
	void docking();
	bool startDocking();	

};

#endif
