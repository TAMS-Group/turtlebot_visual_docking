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
#include <turtlebot_visual_docking/Docking.h>


/**
 * This function returns the amount of in.
 */
float Docking::getAmount(float in){

	if(in < 0 ){
		return (-1)*in;
	}else{
		return in;
	}
}



Docking::Vector2 Docking::normalize(Vector2 vec){

Docking::Vector2 result;
float len = sqrt(vec.x*vec.x+vec.y*vec.y);
if(len == 0.0){
        ROS_ERROR("normalize(): This vector does have a length of zero");
        exit(0);
}else{
 result.x  =  (1/len) * vec.x;
 result.y  =  (1/len) * vec.y;
 return result;
}
}


/**
 * Diese Funktion gibt einen Vector zurück, der um angle Grad 
 * gedreht wurde.
 */
Docking::Vector2 Docking::spinVector(Docking::Vector2 vec,float angle){

float x 	= vec.x;
float y 	= vec.y;
float alpha 	= atan(x/y);

float s 	= alpha + angle;
float R 	= sqrt(x*x+y*y);	 

float y_new 	= R * sqrt(tan(s)*tan(s) + 1);
float x_new     = y_new * tan(s)*tan(s);

Docking::Vector2 v_new;
v_new.x 	= x_new;
v_new.y 	= y_new;  

return v_new;
}



/**
 * This function calculates the angle epsilon described in 
 * equation 11.
 */
float Docking::epsi(float _tx){

return (M_PI/180)*40;

/*
float tx    = 100*_tx;
float tb    = 23.0; //Tagbreite in cm

float p     = 0.30;
float theta = 50 * (M_PI/180);
float a     = tan(M_PI/2);
float b     = tan(theta/2);  

float A     = (tb*(-1)*p)/(2*(p-1)*tx);
float P     = (b+a+A*a-b*A)/(1+a*b*A);
float Q     = (-1*(a*b+A))/(1+a*b*A);

float tane  = (-P/2) + sqrt((P/2)*(P/2) - Q);

return atan(tane);
*/
}

/**
 * This function calculates the docking angle 
 * described in equation 4.
 */
float Docking::docking_angle(){
		tf::StampedTransform transform;
		try{
                listener->lookupTransform("base_link", _tag_name, ros::Time(0), transform);
                }catch (tf::TransformException ex){
                        ROS_ERROR("docking_angle() ::=>%s",ex.what());
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

float Docking::get_direction_angle(){

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
void Docking::get_cameraLink_baseLink(){
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
void Docking::get_optical_frame(){
    try{
        ros::Time begin = ros::Time::now();
        listener->waitForTransform("/camera_rgb_optical_frame","/camera_link",begin,ros::Duration(5.0));
        listener->lookupTransform("/camera_rgb_optical_frame","/camera_link",begin,_transform_optical_cam);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Docking::Show_Quad(tf::Quaternion q){
        tf::Vector3 v           = q.getAxis();
        ROS_INFO("Q[%f , %f ,%f , %f]",v.getX(),v.getY(),v.getZ(),q.getW());

}

/**
 * This function gets the angle alpga_yaw by using the lookupTransform
 * function.
 */
float Docking::get_yaw_angle(){

    tf::StampedTransform yaw;
    try{
        ros::Time begin = ros::Time::now();
        listener->waitForTransform("base_link",_tag_name,begin,ros::Duration(10.0));
        listener->lookupTransform("base_link", _tag_name,begin, yaw);
    }catch (tf::TransformException ex){
        ROS_ERROR("get_yaw_angle %s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
    return  -(M_PI/2 + tf::getYaw(yaw.getRotation()));
}

/**
 * This function gets the position by using the lookupTransform function.
 * It returns a two dimensional Vector (Vector2).
 */
Docking::Vector2 Docking::get_position(){

    tf::StampedTransform transform;
    try{
        ros::Time begin = ros::Time::now();
	ros::Time zero  = ros::Time(0);
        listener->waitForTransform(_tag_name,"base_link",begin,ros::Duration(10.0));
        listener->lookupTransform(_tag_name, "base_link",zero, transform);
    }catch (tf::TransformException ex){
        ROS_ERROR("get_position()  ::=> %s",ex.what());
        ros::Duration(1.0).sleep();
    }
    Docking::Vector2 pos;
    pos.x = transform.getOrigin().x();
    pos.y = transform.getOrigin().z();
    return pos;
}







void Docking::Init(){
    get_cameraLink_baseLink();
    get_optical_frame();
    _start_avg          = false;
    _find_tag 		= false;
    _angle 		= 0.0;
    _TAG_AVAILABLE	= false;
    _try_more 		= true;
    _start_avg		= false;
    _bumper_pressed     = false;
    _avg_pos 		= new Avg(10);
    _avg_dock		= new Avg(10);
    _avg_X 		= new Avg(10);
    _avg_Y		= new Avg(10);
    _avg_yaw		= new Avg(10);
    _odom_pos.x 	= 0.0;
    _odom_pos.y		= 0.0;
}

/**
 * This function starts the frontal docking.
 * Before it does that the avarage arrays are flushed.
 */
void Docking::startFrontalDocking(){
    _start_avg=false;
    _avg_pos->flush_array();
    _avg_dock->flush_array();
    _start_avg = true;
    ros::Duration(1.0).sleep();   
    docking();
}


void Docking::RegisterCallbackFunctions(){
	_sub1 = _n->subscribe("mobile_base/sensors/imu_data",1,&Docking::actual_angle,this);
        _sub2 = _n->subscribe("docking_tags",1,&Docking::get_avg_position_angle,this);
        //_sub3 = _n.subscribe("diagnostics",10,&Docking::get_battery_status,this);
        _sub4 = _n->subscribe("tag_detections",10,&Docking::findTag,this);
        _sub5 = _n->subscribe("mobile_base/sensors/core",1,&Docking::get_charging_status,this);
        _sub6 = _n->subscribe("mobile_base/events/bumper",1,&Docking::get_bumper_status,this);
        _sub7 = _n->subscribe("/mobile_base/sensors/core",1,&Docking::get_ticks,this);
        _sub8 = _n->subscribe("/odom",1,&Docking::get_odom_pos,this);
}
/**
* This constructor is for the use in a node, which uses the launch file: start_docking.launch
*/
Docking::Docking(){

        _n = new ros::NodeHandle();
        _ac  = new MoveBaseClient("move_base", true);
        listener    = new tf::TransformListener();
        _publisher = _n->advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

        //Here we register all _subscribing callback functions
	ros::Duration(6.0).sleep();//wait for the ApriltagDetectionNode
	RegisterCallbackFunctions();

        //The Parameters are read from the launch file : start_docking.launch 
        std::string tag_id;
        if (_n->getParam("/dock/tag_id", tag_id)){
	    _tag_name = "tag_" + tag_id;
	    _tag_id = atoi(tag_id.c_str());
             ROS_INFO(" tag_id = %s",_tag_name.c_str());
        }else{
            ROS_ERROR("No Parameters found!");
            _tag_name = "tag_99";
	    _tag_id = 99;
           // exit(0);
        }

        //Init();
        //exit(0);
        //We wait until tf comes up... 
        ros::Duration(3.0).sleep();
        //All other variables get initilized
        Init();


}


Docking::Docking(ros::NodeHandle* nodeHandle,int tag_id){
     

	_n = nodeHandle;
	_ac  = new MoveBaseClient("move_base", true);
	listener    = new tf::TransformListener();   
        _publisher  = _n->advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
	
        //Here we register all _subscribing callback functions
	ros::Duration(3.0).sleep();//wait for the ApriltagDetectionNode
	RegisterCallbackFunctions();	
        //The Parameters are read. 
	std::string tagid_str = std::to_string(tag_id); 
	_tag_name = "tag_"+tagid_str;
	_tag_id   = tag_id; 
	ROS_ERROR("%s",_tag_name.c_str());
	//We wait until tf comes up... 
        ros::Duration(5.0).sleep();
        //All other variables get initilized
        Init();
}



/**
* This callback function reads the Odometry and saves it into the _odom_pos Vector
*/
void Docking::get_odom_pos(const nav_msgs::Odometry& msg){

	
//	ROS_ERROR("X = %f",msg.pose.pose.position.x);
	_odom_pos.x = msg.pose.pose.position.x;
	_odom_pos.y = msg.pose.pose.position.y; 

	tf::Quaternion *quaternion = new tf::Quaternion(msg.pose.pose.orientation.x,
                                                        msg.pose.pose.orientation.y,
                                                        msg.pose.pose.orientation.y,
                                                        msg.pose.pose.orientation.w);
        tf::Vector3 *pose = new tf::Vector3(tfScalar(msg.pose.pose.position.x),
                                            tfScalar(msg.pose.pose.position.y),
                                            tfScalar(msg.pose.pose.position.z));
        _Odometry_Transform = new tf::Transform(*quaternion,*pose);
           
}



void Docking::get_ticks(const kobuki_msgs::SensorState& msg){

	_ticks_right = msg.right_encoder;
	_ticks_left  = msg.left_encoder;	
}


void Docking::get_bumper_status(const kobuki_msgs::BumperEvent& msg){

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
void Docking::actual_angle(const sensor_msgs::Imu& msg){
        _angle = tf::getYaw(msg.orientation);
}

/**
 * This callback function detects if the robot is in the docking 
 * station and is recharging. The rusult is stored in the variable 
 * _docking_status which is true if the robot is recharging and 
 * false otherwise.  
 */
void Docking::get_battery_status(const diagnostic_msgs::DiagnosticArray& msg){
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

void Docking::get_charging_status(const kobuki_msgs::SensorState& msg){

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
void Docking::get_avg_position_angle(const apriltags_ros::AprilTagDetectionArray& msg){
if(_start_avg){
	apriltags_ros::AprilTagDetection detection;
        int size = msg.detections.size();
//	ROS_ERROR("SIZE=%i",size);
	for(int i = 0; i < size ; i++){
	if(msg.detections[i].id == _tag_id){
            apriltags_ros::AprilTagDetection detect = msg.detections[i];
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
            float xx	= base_tag.getOrigin().x();
            float yy	= base_tag.getOrigin().y();
            float zz	= base_tag.getOrigin().z();
            float yaw   = tf::getYaw(base_tag.getRotation());
	    //float yawyaw= tf::getYaw(p.orientation);
            if( getAmount(xx) <  0.00001){
                ROS_ERROR("Division through zero is not allowed! xx=%f, yy= %f",xx,yy);
                //exit(0);
		return;
            }else{
                float alpha_dock   = atan(yy/xx);
                float alpha_pos    = (alpha_dock - (M_PI/2 + yaw));
		_g_mutex.lock();
                    if(std::isfinite(alpha_pos)){
                        _avg_pos->new_value(alpha_pos);
                        _avg_dock->new_value(alpha_dock);
                        _avg_X->new_value(z);
			_avg_Y->new_value(x);
			_avg_yaw->new_value(yaw + (M_PI/2));
			float avg_pos_angle  	= _avg_pos->avg();
                        float avg_dock_angle 	= _avg_dock->avg();
                        float avg_yaw_angle     = _avg_yaw->avg();
			float avg_position_X 	= _avg_X->avg();
			float avg_position_Y 	= _avg_Y->avg();
			_avg_position_angle 	= avg_pos_angle;
                        _avg_docking_angle  	= avg_dock_angle;
			_avg_position_X 	= avg_position_X;
			_avg_position_Y         = avg_position_Y;
			_avg_yaw_angle		= avg_yaw_angle;
 
                    }
		_g_mutex.unlock();
            }
	}
	}
}
}

/**
 * This callback function sets the variable _TAG_AVAILABLE to true, 
 * when the Apriltag can be seen. Else _TAG_AVAILABLE
 * will be set to false.
 */
void Docking::findTag(const apriltags_ros::AprilTagDetectionArray& msg){
	if(_find_tag){
		apriltags_ros::AprilTagDetection detection;
		int size = msg.detections.size() ;	
		for(int i = 0; i < size ; i++){
			int tag_id = msg.detections[i].id;
			//We are only allowed to take new tag detections!!
			if(_tag_id == tag_id){
				ros::Duration X		= ros::Duration(2.0);
				ros::Time time 		= msg.detections[i].pose.header.stamp;
                                ros::Time now  		= ros::Time::now();
				ros::Duration age 	= now - time;

				if(X > age){
					double secs = age.toSec();
					ROS_INFO("AGE= %f",secs);	
					_TAG_AVAILABLE = true;
				}
			}
		}
	}
}

/**
 * This function moves the robot exact about alpha_rad
 */
void Docking::move_angle(float alpha_rad){

if(alpha_rad != 0.0){
    
    float turn_velocity = 0.5;
    float DT            = getAmount(0.1/turn_velocity);
    float goal_angle    = _angle + alpha_rad;
    int   N             = (int)getAmount((goal_angle / M_PI));
    float rest          = getAmount(goal_angle) - ((float)N * M_PI);
    
    if((rest > 0) && (N > 0)){
        //This is nessesary because the imu is between -180 - 180
        goal_angle = (goal_angle > M_PI) ? -M_PI + rest : goal_angle;
        goal_angle = (goal_angle < -M_PI) ? M_PI - rest : goal_angle;
    }

/*
    ROS_ERROR("N= %i ",N);
    ROS_ERROR("REST = %f",rest);
    ROS_ERROR("ALPHA = %f",(180/M_PI)*alpha_rad);
    ROS_ERROR("GOAL_ANGLE = %f",(180/M_PI)*goal_angle);
*/
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

void Docking::drive_forward(float distance){

ROS_INFO("Start to move forward ...");
// -0.055 is a magic offset to drive exact distances
distance	    = distance - 0.055;

float velocity      = 0.15;
float direction     = distance / getAmount(distance); 
// We actually save the start Transformation
tf::Transform *startTransform = _Odometry_Transform;
tf::Vector3 startPos 	      = startTransform->getOrigin();
float goal 		      = distance;
float driven_x 		      = 0.0;
float driven_y  	      = 0.0;
tf::Vector3 pos_now;
while( direction*goal <= direction * distance ) {
        geometry_msgs::Twist base;
        base.angular.z = 0;
        base.linear.x = direction*velocity;
        _publisher.publish(base);
        // Now we calculate the driven way which is the difference between 
 	// the startTransform and the actual transform
        pos_now 	    = _Odometry_Transform->getOrigin();
	driven_x       	    = pos_now.getX() - startPos.getX();
        driven_y            = pos_now.getY() - startPos.getY();
        goal 	            = direction * sqrt(driven_x*driven_x + driven_y*driven_y);
	//ROS_INFO("The driven distance is: %f",goal);
	// We wait, to reduce the number of sended TwistMessages.
        // If we would not the application crashes after some time.
        ros::Duration(0.3).sleep(); 
    }

geometry_msgs::Twist zero;
_publisher.publish(zero);

}

/**
 * This function drives an exact distance backwards.
 * The parameter distance is given in meter.
 */
void Docking::drive_backward(float distance){
    drive_forward(-distance);
}


/**
 * This function dirves to the given coordinates.
 * This function only works if the robot does know its position.
 * 
 */ 

void Docking::move_to(float x, float y){

	    //MoveBaseClient ac("move_base", true);
	    
	    while(!_ac->waitForServer(ros::Duration(5.0))){
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
	    _ac->sendGoal(goal);

	    _ac->waitForResult();
	    ROS_INFO("Goal has been reached!");

}


/**
 * This function turns the robot counter clock-wise until the 
 * Apriltag can be seen by the camera.
 */
void Docking::searchTag(){
    ROS_INFO("searching Tag ...");
    _find_tag = true;// switch on the callback function findTag
   ros::Duration(1.5).sleep(); //wait until the tag can be recognized
   while(!_TAG_AVAILABLE){
	geometry_msgs::Twist base; 
	base.linear.x   = 0.0;
       	base.angular.z  = 0.8;//5.0 * (M_PI/180);
       	_publisher.publish(base);
  	ros::Duration(0.5).sleep();
   }
  _find_tag = false; // switch off the callback function findTag
}

/**
 * This function turns the robot in order to have an 
 * resulting yaw angle of zero deg.
 */
void Docking::adjusting(){
	ROS_INFO("Adjusting position ...");
	startReadingAngle();
	float yaw = _avg_yaw_angle;
	ROS_INFO("Adjusting Yaw = %f",yaw);
	stopReadingAngle();
	move_angle(yaw);
}

/**
* The robot turns until the docking asngle is smaller than epsilon.
*/
void Docking::watchTag(){
   startReadingAngle();
   float epsilon = 3.0 ;
   while(getAmount((180/M_PI)*_avg_docking_angle) > epsilon){
        geometry_msgs::Twist base;
        base.linear.x   = 0.0;
        base.angular.z  = 0.2;//5.0 * (M_PI/180);
        _publisher.publish(base);
        ros::Duration(0.5).sleep();
	//ROS_INFO("Angle = %f",(180/M_PI)*_avg_docking_angle);
    }
    stopReadingAngle();
}




/**
 * This function does the linear Approach described in section 6.2
 */
void Docking::linearApproach(){
    adjusting();
    ROS_INFO("Begin linear approach");
    Docking::Vector2 pos;

    startReadingAngle();
    float alpha         = _avg_position_angle;
    pos.x 		= _avg_position_X;
    pos.y 		= _avg_position_Y;  
    stopReadingAngle();

    float alpha_deg     = (180/M_PI)*alpha;
    float epsilon_rad   = epsi(pos.x);
    float epsilon_deg   = (180/M_PI)*epsilon_rad;
    float e 		= getAmount(epsilon_rad);

    float beta 		= (M_PI/2) - epsilon_rad;
    float d 		= 100*pos.y; //in cm
    float way		= sqrt( (d*d) + (1 + (tan(beta)*tan(beta))) );
    way 		= way/100; 

    ROS_INFO("POS_ANGLE= %f",alpha_deg);
    ROS_INFO("EPSILON= %f",epsilon_deg);	
    ROS_INFO("Way = %f", way);    
    
     if(alpha < 0.0 ) { //TURN RIGHT
            ROS_INFO("Turning right with epsilon= %f deg",epsilon_deg);
            move_angle(-e);
	    drive_forward(way);	
     	    move_angle(e);   
	}

    if(alpha > 0.0){ //TURN LEFT
            ROS_INFO("Turning left with epsilon= %f deg",epsilon_deg);
            move_angle(e);
            drive_forward(way);	
	    move_angle(-e);
	}
      this->adjusting();
      ROS_INFO("Start Docking");

	startReadingAngle();
	this->docking();	
}

void  Docking::startReadingAngle(){
	_start_avg = false;
        _avg_pos->flush_array();
        _avg_dock->flush_array();
	_avg_yaw->flush_array();
	_avg_X->flush_array();
	_avg_Y->flush_array();
        _start_avg = true;
        ros::Duration(5.0).sleep();
	}

void Docking::stopReadingAngle(){
	_start_avg = false;
}


/**
 * This function does the positioning described in section 6.1
 */
void Docking::positioning(){
	ROS_INFO("Staring positioning...");
	startReadingAngle();
		float a_pos_rad             = _avg_position_angle;
		float a_pos_deg		    = (180/M_PI)*_avg_position_angle;
		Docking::Vector2 pos;
		pos.x = _avg_position_X;
		pos.y = _avg_position_Y;
	stopReadingAngle();
	//ROS_ERROR("Angle read with :  %f deg",a_pos_deg);
	//ROS_ERROR("Angle read with : %f rad",a_pos_rad);
	//ROS_ERROR("Pos_X read with : %f",_avg_position_X);
	//ROS_ERROR("Pos_Y read with : %f",_avg_position_Y);

	if( getAmount(a_pos_deg)  < 6.0){
	    ROS_INFO("Start frontal docking without positioning...");
	    ROS_INFO("Angle = %f",_avg_position_angle);
            startReadingAngle();
	    docking(); 
        }else{
            float alpha_yaw 	= get_yaw_angle();
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
            this->searchTag();
            ros::Duration(1.0).sleep();
	    this->adjusting();
            ros::Duration(1.0).sleep();
            
            startReadingAngle();
            float a_pos_rad = getAmount(_avg_position_angle);
            stopReadingAngle();
            
            a_pos_deg = getAmount((180/M_PI) * a_pos_rad);
            //ROS_ERROR("A_POS_DEG = %f",a_pos_deg); 
            if(a_pos_deg < 20.0){
               if(a_pos_deg < 10.0){
                    startReadingAngle();
                    ros::Duration(1.0).sleep();
                    docking();
                }else{  


		    // We should only allow the linear approach if the robot is 
		    // far enough away from the docking station
		   startReadingAngle();
		   float X       = getAmount(_avg_position_X);
		   float epsilon = epsi(X); 
		   float phi     = (M_PI/2) - epsilon;
		   float D       = tan(phi) * getAmount(_avg_position_Y);
 		   stopReadingAngle();
		
	           ROS_INFO("D = %f",D);
		   ROS_INFO("X = %f",X);
		   ROS_INFO("d = %f",X-D);		
		   // <u>After</u> the linear approch the robot should be at least 
		   // 10cm away from the docking station we have to caculate this 
		   // distance to prevent crashing into the docking station while 
		   // we perform the linear approach, which does not have 
		   // a colision control.
		   if( (X-D) > 0.10 ){	
		    		linearApproach();
			}else{
				// if we cannot do the linear approach, because we are 
				// to close to the docking station we simply try to dock.
				//  Actually is should never come to this.
				startReadingAngle();
				docking();
			}
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
void Docking::docking(){
//    ROS_INFO("Startring frontal docking...");
    geometry_msgs::Twist      base;
    float DELTA             = 2.0;
    _try_more               = false;
	Docking::Vector2 pos;
	pos.x 		= _avg_position_X;
	pos.y 		= _avg_position_Y;
	//Docking::Vector2 pos    = get_position();
     	float alpha_deg         = (180/M_PI)*_avg_docking_angle;
	//stopReadingAngle();
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
        if(pos.x > 0.60 ){
		base.linear.x = 0.15;// We should drive very slow
	}else{
		base.linear.x = 0.02;// If we are near enough we drive even slower
	}
        }

	//ROS_INFO("POS.x = %f",pos.x);
	// Finally publish the __base_cmd the __base_cmd
        // The Robot should stop when the battery is recharging
        // The Robot should drive backwards, when the bumper was triggered

        if(_docking_status){
		stopReadingAngle();
		ros::Duration(1.0).sleep();
		return;  //exit(0); // On this point the docking was sucessfull
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
bool Docking::startDocking(){
	//move_to(_TURTLEBOT_PRE_DOCKING_POSE_X,_TURTLEBOT_PRE_DOCKING_POSE_Y);
	ros::Duration(1.0).sleep();
	searchTag();
	watchTag();
	ros::Duration(2.0).sleep();	
	positioning();
	
	adjusting();
	ros::Duration(2.0).sleep();
	startReadingAngle();
	docking();
 	return true;
}	
