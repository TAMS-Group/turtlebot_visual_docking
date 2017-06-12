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
#include <tf/LinearMath/Vector3.h>
#include "avg.cpp"
#include <mutex>

class Docking {

private:

struct Vector2{         
    float x;
    float y;
  };
  
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

tf::TransformListener   *listener;
ros::Publisher          _publisher;
ros::Subscriber 	_sub1;
ros::Subscriber		_sub2;
ros::Subscriber 	_sub3;
ros::Subscriber		_sub4;
ros::Subscriber		_sub5;
ros::Subscriber		_sub6;
double 			_angle;
ros::NodeHandle		_n;
tf::StampedTransform    _transform_cam_base;
tf::StampedTransform    _transform_optical_cam;
bool			_find_tag;
float 			_start_pos_x;
float                   _start_pos_y;
float                   _start_pos_z;
bool 			_docking_status;// Is true when robot is loading energy
bool 			_try_more;
bool 			_start_avg;	
bool 			_TAG_AVAILABLE;		
Avg			*_avg_pos;
Avg			*_avg_dock;
std::mutex 		_g_mutex; 
float 			_avg_position_angle;
float 			_avg_docking_angle;
std::string 		_tag_name;
double 			_STOP_DISTANCE;
bool                    _bumper_pressed;


/**
 * This function returns the amount of in.
 */
float getAmount(float in){

	if(in < 0 ){
		return (-1)*in;
	}else{
		return in;
	}
}

/**
 * Diese Funktion gibt einen Vector zurück, der um angle Grad 
 * gedreht wurde.
 */
Vector2 spinVector(Vector2 vec,float angle){

float x 	= vec.x;
float y 	= vec.y;
float alpha 	= atan(x/y);

float s 	= alpha + angle;
float R 	= sqrt(x*x+y*y);	 

float y_new 	= R * sqrt(tan(s)*tan(s) + 1);
float x_new     = y_new * tan(s)*tan(s);

Vector2 v_new;
v_new.x 	= x_new;
v_new.y 	= y_new;  

return v_new;
}



/**
 * This function calculates the angle epsilon described in 
 * equation 11.
 */
float epsi(float _tx){

float tx    = 100*_tx;
float tb    = 7.7; //Tagbreite in cm

float p     = 0.25;
float theta = 50 * (M_PI/180);
float a     = tan(M_PI/2);
float b     = tan(theta/2);  

float A     = (tb*(-1)*p)/(2*(p-1)*tx);
float P     = (b+a+A*a-b*A)/(1+a*b*A);
float Q     = (-1*(a*b+A))/(1+a*b*A);

float tane  = (-P/2) + sqrt((P/2)*(P/2) - Q);

return atan(tane);

}

/**
 * This function calculates the docking angle 
 * described in equation 4.
 */
float docking_angle(){
		tf::StampedTransform transform;
		try{
                listener->lookupTransform("base_link", _tag_name, ros::Time(0), transform);
                }catch (tf::TransformException ex){
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                }
                float map_tag_x = transform.getOrigin().x();
                float map_tag_y = transform.getOrigin().y();
                float map_tag_z = transform.getOrigin().z();

		return (map_tag_y/map_tag_x);

}

/**
 * This function gives the angle in rad between the x-axis and the transform. 
 * The orign of the coordinate-system is the April-Tag. Thus the apriltag must be
 * visible when this function is used. 
 */

float get_direction_angle(){

	tf::StampedTransform transform;
           
	try{        

	  ros::Time begin = ros::Time::now();
          listener->waitForTransform(_tag_name,"base_link",begin,ros::Duration(5.0));
          listener->lookupTransform(_tag_name, "base_link",ros::Time(0), transform);

        }catch (tf::TransformException ex){
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
        }

        float tag_x         = transform.getOrigin().x();
        float tag_y         = transform.getOrigin().z();

        float wd_rad        = atan(tag_x/tag_y);
	return wd_rad;
}


/**
 * This function gets the transform between the 
 * camera and the base_link.
 * The result is stored in _trcamera_rgb_optical_frameansform_cam_base
 */
void get_cameraLink_baseLink(){
    try{
        ros::Time begin = ros::Time::now();
        listener->waitForTransform("camera_link","base_link",begin,ros::Duration(5.0));
        listener->lookupTransform("camera_link","base_link",begin,_transform_cam_base);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }


}


/**
 * This function gets the transform betwwen the 
 * camera_rgb_optical_frame and the camera_link.
 * The result is stored in _transform_optical_cam
 */
void get_optical_frame(){
    try{
        ros::Time begin = ros::Time::now();
        listener->waitForTransform("/camera_rgb_optical_frame","/camera_link",begin,ros::Duration(5.0));
        listener->lookupTransform("/camera_rgb_optical_frame","/camera_link",begin,_transform_optical_cam);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Show_Quad(tf::Quaternion q){
        tf::Vector3 v           = q.getAxis();
        ROS_INFO("Q[%f , %f ,%f , %f]",v.getX(),v.getY(),v.getZ(),q.getW());

}

/**
 * This function gets the angle alpga_yaw by using the lookupTransform
 * function.
 */
float get_yaw_angle(){

    tf::StampedTransform yaw;
    try{
        ros::Time begin = ros::Time::now();
        listener->waitForTransform("base_link",_tag_name,begin,ros::Duration(5.0));
        listener->lookupTransform("base_link", _tag_name,begin, yaw);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
    return  -(M_PI/2 + tf::getYaw(yaw.getRotation()));
}

/**
 * This function gets the position by using the lookupTransform function.
 * It returns a two dimensional Vector (Vector2).
 */
Vector2 get_position(){

    tf::StampedTransform transform;
    try{
        ros::Time begin = ros::Time::now();
        listener->waitForTransform(_tag_name,"base_link",begin,ros::Duration(5.0));
        listener->lookupTransform(_tag_name, "base_link",begin, transform);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    Vector2 pos;
    pos.x = transform.getOrigin().x();
    pos.y = transform.getOrigin().z();
    return pos;
}

void Init(){
    get_cameraLink_baseLink();
    get_optical_frame();
    _start_avg          = false;
    _find_tag 		= true;
    _angle 		= 0.0;
    _TAG_AVAILABLE	= false;
    _try_more 		= true;
    _start_avg		= false;
    _bumper_pressed     = false;
    _avg_pos 		= new Avg(40);
    _avg_dock		= new Avg(10);
}

public: 


/**
 * This function starts the frontal docking.
 * Before it does that the avarage arrays are flushed.
 */
void startFrontalDocking(){
    _start_avg=false;
    _avg_pos->flush_array();
    _avg_dock->flush_array();
    _start_avg = true;
    ros::Duration(1.0).sleep();   
    docking();
}


Docking(){
        ros::NodeHandle _n;
        listener    = new tf::TransformListener();
    
        _publisher = _n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
	
        //Here we register all _subscribing callback functions
        _sub1 = _n.subscribe("mobile_base/sensors/imu_data",1,&Docking::actual_angle,this);	
	_sub2 = _n.subscribe("tag_detections",1,&Docking::get_avg_position_angle,this);		
	//_sub3 = _n.subscribe("diagnostics",10,&Docking::get_battery_status,this);
	_sub4 = _n.subscribe("tag_detections",10,&Docking::findTag,this);
	_sub5 = _n.subscribe("mobile_base/sensors/core",1,&Docking::get_charging_status,this);
        _sub6 = _n.subscribe("mobile_base/events/bumper",1,&Docking::get_bumper_status,this);

        //The Parameters are read. 
	std::string tag_id;
	if (_n.getParam("/dock/tag_id", tag_id)){
            _tag_name = "tag_" + tag_id;
  	     //ROS_INFO(" tag_id = %s",_tag_name.c_str());
	}else{
            ROS_ERROR("No Parameters found!");
            exit(0);
	}
        if (!_n.getParam("/dock/stop_distance", _STOP_DISTANCE)){
            ROS_ERROR("No Parameters for Stop_Distance found!");
            exit(0);
        }
	//exit(0);
	//We wait until tf comes up... 
        ros::Duration(5.0).sleep();
        //All other variables get initilized
        Init();
}


void get_bumper_status(const kobuki_msgs::BumperEvent& msg){

int status_all = msg.state;

if(status_all==1){
        _bumper_pressed = true;
}

}


/**
 * This callback function read the rotation angle, 
 * from the IMU. The value is stored in the Varibale 
 * _angle.
 */
void actual_angle(const sensor_msgs::Imu& msg){
        _angle = tf::getYaw(msg.orientation);
}

/**
 * This callback function detects if the robot is in the docking 
 * station and is recharging. The rusult is stored in the variable 
 * _docking_status which is true if the robot is recharging and 
 * false otherwise.  
 */
void get_battery_status(const diagnostic_msgs::DiagnosticArray& msg){
    int s = msg.status.size();
    if(s >= 1){
        int size =  msg.status[0].values.size();
            for(int i = 0; i < size; i++){
                std::string key = msg.status[0].values[i].key;
                    if(key == "Source"){
                        std::string value = msg.status[0].values[i].value;
                            if(value == "None"){
                                _docking_status = false;
				}else{
                                _docking_status = true;
                            }
			}
		}
	}
}

/**
 * This callback function detects if the robot is in the docking
 * station and does recharging.
 */

void get_charging_status(const kobuki_msgs::SensorState& msg){

int status = msg.charger;

//ROS_ERROR("status : %i",status);


if(status == 6 ){
        _docking_status = true;
}else{
        _docking_status = false;
}


}

/*
 * Diese callback funktion liest 10 mal die Pose aus und mittelt den Winkel um 
 * Störanfälliges rauschen zu verhindern.
 */
void get_avg_position_angle(const apriltags_ros::AprilTagDetectionArray& msg){
if(_start_avg){
	apriltags_ros::AprilTagDetection detection;
        int size = msg.detections.size() ;
	if(size == 1){
            apriltags_ros::AprilTagDetection detect = msg.detections[0];
            int id                                  = detect.id;
            double siz                              = detect.size;
            geometry_msgs::PoseStamped pose         = detect.pose;
            geometry_msgs::Pose p                   = pose.pose;
            geometry_msgs::Point point              = p.position;
            float x = point.x;
            float y = point.y;
            float z = point.z;
            tf::Quaternion *optical_tag_quad = new tf::Quaternion(p.orientation.x,
                                                                  p.orientation.y,
                                                                  p.orientation.z,
                                                                  p.orientation.w);
            tf::Vector3 *optical_tag_origin = new tf::Vector3(tfScalar(x),
                                                              tfScalar(y),
                                                              tfScalar(z));   
            tf::Transform *optical_tag_trans = new tf::Transform(*optical_tag_quad,*optical_tag_origin);	
            tf::Transform tag_cam =  optical_tag_trans->inverse() * _transform_optical_cam;
            tf::Transform tag_base = tag_cam * _transform_cam_base;
            tf::Transform base_tag = tag_base.inverse();
            // The transform CAM -> BASE is constant in all coordinate systems
            float _x	= base_tag.getOrigin().x();
            float _y	= base_tag.getOrigin().y();
            float _z	= base_tag.getOrigin().z();
            float yaw   = tf::getYaw(base_tag.getRotation());
            if( getAmount(_x) <  0.0001){
                ROS_ERROR("Division through zero is not allowed!");
                exit(0);
            }else{
                float alpha_dock   = atan(_y/_x);
                float alpha_pos    = alpha_dock - (M_PI/2 + yaw);
                _g_mutex.lock();
                    if(std::isfinite(alpha_pos)){
                        _avg_pos->new_value(alpha_pos);
                        _avg_dock->new_value(alpha_dock);
                        float avg_pos_angle  = _avg_pos->avg();
                        float avg_dock_angle = _avg_dock->avg();
                        _avg_position_angle = avg_pos_angle;
                        _avg_docking_angle  = avg_dock_angle;
                    }
		_g_mutex.unlock();
            }
	}
}
}

/**
 * This callback function sets the variable _TAG_AVAILABLE to true, 
 * when the Apriltag can be seen. Else _TAG_AVAILABLE
 * will be set to false.
 */
void findTag(const apriltags_ros::AprilTagDetectionArray& msg){
	if(_find_tag){
		apriltags_ros::AprilTagDetection detection;
		int size = msg.detections.size() ;
		_TAG_AVAILABLE = (size == 1);
	}
}

/**
 * This function moves the robot exact about alpha_rad
 */
void move_angle(float alpha_rad){

if(alpha_rad != 0.0){
    
    float turn_velocity = 0.5;
    float DT            = getAmount(0.1/turn_velocity);
    float goal_angle    = _angle + alpha_rad;
    int   N             = (int)getAmount((goal_angle / M_PI));
    float rest          = goal_angle - ((float)N * M_PI);
    
    if((rest > 0) && (N > 0) ){
        //This is nessesary because the imu is between -180 - 180
        goal_angle = (goal_angle > M_PI) ? -M_PI + rest : goal_angle;
        goal_angle = (goal_angle < -M_PI) ? M_PI - rest : goal_angle;
    }
    float rad_velocity 	= (alpha_rad < 0) ? (-1)*turn_velocity : turn_velocity;
    bool weiter 	= true; 
    float alpha_old 	= 0;
    float alpha_new	= _angle;
    while(weiter){
        geometry_msgs::Twist base;
        base.linear.x 	= 0.0;
        base.angular.z 	= rad_velocity;
        _publisher.publish(base);
        //We wait, to reduce the number of sended TwistMessages,
        //If we would not the application crashes after some time.
	ros::Duration(DT).sleep();
        float epsilon 	= goal_angle - _angle;
        epsilon 	= getAmount(epsilon);  
        weiter = epsilon  > 0.1 ;
    }
}
}

/**
 * This function drives the robot forward. The distance in [m] 
 * is given as a parameter, but the velocity should be fixed
 * 
 * TODO: Try the callibration with different velocitys.
 */
void drive_forward(float d){

    float velocity      = 0.15;
    float distance	= getAmount(d);
   
    float direction     = (d < 0) ? -1.0 : 1.0;
    
    float time_to_wait  = distance / velocity;
    //Calculating the ERROR
    float  ERROR        = 8.92 * log(distance*100) - 26.515;
    float  dt           = ERROR / (velocity*100);
    if(dt > 0){
        //USING NORMAL ERROR EQUATION
        time_to_wait    = time_to_wait + dt;
    }else{
        //USING ERROR EQUATION FOR SMALL DISTANCES");
        velocity        = 0.05;
        time_to_wait    = distance / velocity;
        ERROR           = 0.395 * (distance*100) - 0.079;
        dt              = ERROR / (velocity*100);
        time_to_wait    = time_to_wait + dt;
    }
    ros::Time startTime = ros::Time::now();
    while(ros::Time::now() - startTime < ros::Duration(time_to_wait)){
        geometry_msgs::Twist base;
        base.angular.z = 0;
        base.linear.x = direction*velocity;
        _publisher.publish(base);
        //We wait, to reduce the number of sended TwistMessages.
        //If we would not the application crashes after some time.
        ros::Duration(0.5).sleep(); 
    }
}

/**
 * This function drives an exact distance backwards.
 * The parameter distance is given in meter.
 */
void drive_backward(float distance){
    drive_forward(-distance);
}


/**
 * This function dirves to the given coordinates.
 * This function only works if the robot does know its position.
 * 
 */ 

void move_to(float x, float y){

    MoveBaseClient ac("move_base", true);
    
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    
    // Actually we use the topic map because we need a fixpoint, 
    // which does not change over time
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    goal.target_pose.pose.orientation.w = 1.0 ;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();
    ROS_INFO("Ausgangsposition erreicht");

}



/**
 * This function should memorize the actuall position. 
 * The position is saved in the variables 
 * _start_pos_x, _start_pos_y, and _start_pos_z.
 */
void RememberPosition(){
// Lese Position aus dem base_link
    tf::StampedTransform transform;
    try{
        ros::Time begin = ros::Time::now();
        listener->waitForTransform("base_link",_tag_name,begin,ros::Duration(5.0));
        listener->lookupTransform("base_link", _tag_name,ros::Time(0), transform);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    _start_pos_x = transform.getOrigin().x();
    _start_pos_y = transform.getOrigin().y();
    _start_pos_z = transform.getOrigin().z();
}


/**
 * This function turns the robot counter clock-wise until the 
 * Apriltag can be seen by the camera.
 */
void searchTag(){
    _find_tag = true;// switch on the callback function findTag
    while(!_TAG_AVAILABLE){
        geometry_msgs::Twist base; 
        base.linear.x   = 0.0;
        base.angular.z  = 0.5;//5.0 * (M_PI/180);
        _publisher.publish(base);
        ros::Duration(0.5).sleep();
    }
    _find_tag = false; // switch off the callback function findTag
}

/**
 * This function turns the robot in order to have an 
 * resulting yaw angle of zero deg.
 */
void adjusting(){
	float yaw = get_yaw_angle();
	move_angle(-yaw);
}

/**
 * This function does the linear Approach described in section 6.2
 */
void linear_approach(){
    adjusting();
    ROS_INFO("Begin fine positioning");
    Vector2 pos 		= get_position();
    Vector2 t;
    t.x = 1.0;
    t.y = 0.0;
    startReadingAngle();
    float alpha         = _avg_position_angle;
    stopReadingAngle();
    float alpha_deg    = (180/M_PI)*alpha;
    float epsilon       = epsi(pos.x);
    Vector2 vec2 	= spinVector(t,epsilon);
    Vector2               vec_n;//Vector vec2 normieren 
    vec_n.x             =  1/(sqrt(vec2.x*vec2.x+vec2.y*vec2.y)) * vec2.x;
    vec_n.y             =  1/(sqrt(vec2.x*vec2.x+vec2.y*vec2.y)) * vec2.y;
    //Bedingung die zu prüfen ist
    float d = -1*((t.y * vec_n.x)/(vec_n.y)) - t.x;
    d = (-1)*d;
    ROS_INFO("DISTANCE = %f",d);
    ROS_INFO("WINKEL= %f",(180/M_PI)*alpha);
    ROS_INFO("EPSILON= %f",epsilon*(180/M_PI));
    if (d < 0.10){
        ROS_INFO("Distance to short --> Repositioning ");
        //Positioniere dich neu
        move_angle(M_PI);
        drive_forward(0.50);
        move_angle(-M_PI);
        ros::Duration(3.0).sleep();
        linear_approach();
        return;
    }else{
        startReadingAngle();
        float e = getAmount(epsilon);
        if(alpha < 0.0 ) { //TURN RIGHT
            float deg = (180/M_PI)*epsilon;
            ROS_INFO("Turning right with alpha2= %f",deg);
            move_angle(-e);	
            float alpha_neu = -360;
            while(alpha_neu < 0.0){
                geometry_msgs::Twist    base;
                base.angular.z  = 0;
                base.linear.x   = 2*0.035;
                alpha_neu       = (180/M_PI)*(_avg_position_angle);
                _publisher.publish(base);
            }
        }
        if(alpha > 0.0){ //TURN LEFT
            float deg = (180/M_PI) * epsilon;
            ROS_INFO("Turning left with alpha2= %f",deg);
            move_angle(e);
            float alpha_neu = 360;
            while(alpha_neu > 0.0){
                geometry_msgs::Twist    base;
                base.angular.z  = 0;
                base.linear.x   = 2*0.035;
                alpha_neu       = (180/M_PI)*(_avg_position_angle);
                _publisher.publish(base);
                }
        }
        ROS_INFO("STOP");
        stopReadingAngle();
        if(alpha < 0.0){//TURN LEFT AGAIN
                move_angle(e);
        }
        if(alpha > 0.0){ //TURN RIGHT AGAIN
                move_angle(-e);
        }
        this->adjusting();
        startReadingAngle();
        docking();
    }
}

void  startReadingAngle(){
	_start_avg = false;
        _avg_pos->flush_array();
        _avg_dock->flush_array();
        _start_avg = true;
        ros::Duration(2.0).sleep();
}

void stopReadingAngle(){
	_start_avg = false;
}

/**
 * This function does the positioning described in section 6.1
 */
void positioning(){
	ROS_INFO("Starte Positioning");
	RememberPosition();
	Vector2 pos 	      	    = get_position();
	startReadingAngle();
	float a_pos_rad             = _avg_position_angle;
	stopReadingAngle();
	if( getAmount(_avg_position_angle) < 11.0*(M_PI/180)){
            _start_avg = false;
            ros::Duration(1.0).sleep();
            _avg_pos->flush_array();
            _avg_dock->flush_array();
            _start_avg = true;
            ros::Duration(1.0).sleep();
            docking(); 
        }else{
            float alpha_yaw 	= get_yaw_angle();
            float a_pos_deg     = (180/M_PI) * a_pos_rad;
            float beta_rad      = 0.0;
            float way           = 0.0;
            if(a_pos_deg < 0.0){// Now the robot has to turn right
                    ROS_INFO("Turning right!");
                    beta_rad = ((M_PI/2) + alpha_yaw);
                    way = sin(a_pos_rad) * sqrt(pos.x*pos.x+pos.y*pos.y);
                    this->move_angle((-1)*beta_rad);
            }else{
                if(a_pos_deg > 0.0){// Now the robot has to turn left
                    ROS_INFO("Turning left");
                    beta_rad  = (M_PI/2) - alpha_yaw;
                    way = sin(a_pos_rad) * sqrt(pos.x*pos.x+pos.y*pos.y);
                    this->move_angle(beta_rad);
                }
            }
            ROS_INFO("Weglaenge = %f",way);
            ROS_INFO("Fahre los!");
            // Drive in the frontal Position 
            drive_forward(getAmount(way));
            // Now the robot should be in the frontal Position 
            // he must turn now 
            if(a_pos_deg > 0.0){
                this->move_angle((-1)*M_PI/2);
            }else{
                this->move_angle((M_PI/2));
            }
            ros::Duration(1.0).sleep();
            this->adjusting();
            ros::Duration(1.0).sleep();
            
            startReadingAngle();
            float a_pos_rad = getAmount(_avg_position_angle);
            stopReadingAngle();
            
            a_pos_deg = (180/M_PI) * a_pos_rad;
            
            if(a_pos_deg < 20.0){
                if(a_pos_deg < 10.0){
                    startReadingAngle();
                    ros::Duration(1.0).sleep();
                    docking();
                }else{
                    linear_approach();
                }
            }else{
                ROS_ERROR("RESTART_POSITIONING");
                positioning();
            }
        }
}

/**
 * This function does the frontal docking, which is 
 * described in section 6.3
 */
void docking(){
    geometry_msgs::Twist      base;
    float DELTA             = 2.0;
    _try_more                = false;
    Vector2 pos             = get_position();
    float alpha_deg         = (180/M_PI)*_avg_docking_angle;
    // if the angle becomes bigger than 15 deg 
    if((alpha_deg > 15.0) && _try_more){
        ROS_ERROR("DOCKING WILL FAIL");
        ROS_ERROR("RESTARTING THE POSITIONING...");
        positioning();
        return;
    }else{
        base.linear.x   = 0.0;
        if (alpha_deg > DELTA){
            base.angular.z = 0.27; // slow but it works
        }else if (alpha_deg < -DELTA) {
            base.angular.z = -0.27;
        }else{
            base.angular.z = 0;
        if(pos.y > _STOP_DISTANCE + 0.20 ){
		base.linear.x = 0.1;// We should drive very slow
	}else{
		base.linear.x = 0.02;
	}
        }

	// Finally publish the __base_cmd the __base_cmd
        // The Robot should stop when the battery is recharging
        // The Robot should drive backwards, when the bumper was triggered

        if(_docking_status){
		exit(0); // On this point the docking was sucessfull
	}else{
        if(!_bumper_pressed){
            _publisher.publish(base);
            ros::Duration(0.5).sleep();
            docking();
         }else{
            ROS_INFO("Try once more...\n");
            this->drive_backward(0.55);
            this->adjusting();
            _bumper_pressed = false;
            docking();
         }
        }
    }
}


/**
 * This function starts the docking.
 */
void startDocking(){

	searchTag();
	ros::Duration(2.0).sleep();
	positioning();
}	

};


int main(int argc, char **argv){

        ROS_INFO("Docking is started!");
	ROS_INFO("Hopefully the robot knows where he is!");
        ros::init(argc,argv,"follow_tag");
        ros::AsyncSpinner spinner(5);
        spinner.start();
        Docking *d = new Docking();
       
	//We have to drive to [2.867, 1.030, 0.010]
	d->move_to(2.867 , 1.030);
 
	d->startDocking();

	ros::spin();
        return 0;
}
