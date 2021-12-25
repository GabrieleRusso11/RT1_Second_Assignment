#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "robot_controller/Velocity.h"
#include <iostream>

using namespace std;

/*
ranges will contain all distances detected by the laser scan sensor 
(between 0 and 180 degrees),whereas right_side, left_side and front_side
are ranges subsection. 
front_side contains all frontal distances between 
80 and 100 degrees.
right_side contains all lateral distances between 0 and 80 degrees.
left_side contains all lateral distances between 100 and 180 degrees                                                           
*/
float ranges[720], right_side[320], left_side[320], front_side[80];

// th and dan_th are threshold values, th is a soft threshold whereas dan_th (dangerous threshold)
// is a hard threshold which indicates that the robot is too close to the wall 
float dan_th = 0.45;
float th = 2.0;

// normal speed : is the robot speed when it is far enough from the walls
float speed_n = 0.5;

// publisher declaration
ros::Publisher pub;

// velocityControl is a function that is called when a client sends a request to the 
// custom service /velocity_control that takes as request a command by the user
// and respond with an increment or a decrement of the robot speed.
bool velocityControl(robot_controller::Velocity::Request &req, robot_controller::Velocity::Response &res){

	if(req.command == 'i'){
		res.speed += 0.5; 
	}
	else if(req.command == 'd'){
		res.speed -= 0.5;
	}
	
	speed_n += res.speed;
	return true;
}

// min_right_side is a function that takes as input parameter the ranges array
// ,computes the minimum value among the right lateral distances contained in 
// the right_side array and return the minimum value as output.
float min_right_side(float * r){
	float min_r = 100;
	int j = 0;
	for(int i = 0; i < 320; i++){
		right_side[j] = r[i];
		if(right_side[j] < min_r){
			min_r = right_side[j];
		}
		j++;
	}
	return min_r;
}

// min_left_side is a function that takes as input parameter the ranges array
// ,computes the minimum value among the left lateral distances contained in 
// the left_side array and returns the minimum value as output.
float min_left_side(float * r){
	float min_l = 100;
	int j = 0;
	for(int i = 400; i < 720; i++){
		left_side[j] = r[i];
		if(left_side[j] < min_l){
			min_l = left_side[j];
		}
		j++;
	}
	return min_l;
}

// min_front_side is a function that takes as input parameter the ranges array
// ,computes the minimum value among the frontal distances contained in 
// the front_side array and returns the minimum value as output.
float min_front_side(float * r){
	float min_f = 100;
	int j=0;
	for(int i = 320; i < 400; i++){
		front_side[j] = r[i];
		if(front_side[j] <  min_f){
			min_f = front_side[j];
		}
		j++;
	}
	return min_f;
}

// drive is a function that takes as input parameters the desired 
// robot linear speed and the robot angular speed and then
// using the topic /cmd_vel it publishes the linear and angular robot speed
void drive(float speed_l, float speed_a){

	geometry_msgs::Twist robot_velocity;
	robot_velocity.linear.x = speed_l;
	robot_velocity.angular.z = speed_a;
	pub.publish(robot_velocity);

}

// turn_only is a function that takes as input parameters the desired 
// robot angular speed and then using the topic /cmd_vel it publishes
// the linear and angular robot speed (the linear speed is set to zero, 
// because in this case the robot have to turn only).
void turn_only(float speed){

	geometry_msgs::Twist robot_velocity;
	robot_velocity.linear.x = 0;
	robot_velocity.angular.z = speed;
	pub.publish(robot_velocity);

}

// avoid_walls is a function used to avoid that the robot crashes on the walls
// of the circuit. It takes as input parameters the front, left and right minimum
// values, computed by the min_left_side, min_right_side amd min_front_side
// functions and using this values it checks if the robot crosses the threshold
// ,for instance, firstly it checks if the front min value is less than th, 
// if it isn't ,the robot go on ,otherwise it checks if the left min
// is less than the right min and vice versa. If the left min is less than the right min 
// it means that the nearest wall is on the left, so at this point it checks if the left min
// is less than the dangerous threshold (if left < dan_th it means that the wall is too close), 
// in the case left < dan_th is true, the robot turn only on the right (with no linear velocity)
// , whereas in the case left < dan_th is false, the robot turn on the right with a linear speed
// different to zero. In both cases the speed is computed as sum of the left and right minimum
// value in order to have a speed that changes with the distance from the walls. 
void avoid_walls(float front, float left, float right){

	float speed_d = left + right;

	if(front > th){
		drive(speed_n,0.0);
	}
	else if(front < th){
		if(left < right){
			cout<<"turn right"<<endl;
			if(left < dan_th){
				cout<<"too close to the wall"<<endl;
				turn_only(-speed_d);
			}
			else{
				drive(speed_d, -1.7);
			}
		}
		else if(right < left){
			cout<<"turn left"<<endl;
			if(right < dan_th){
				cout<<"too close to the wall"<<endl;
				turn_only(speed_d);
			}
			else{
				drive(speed_d, 1.7);
			}
		}
	}
}

// the callback function robotCallback is a function that is called each time
// a message is published on the /base_scan topic, so every time the laser
// scan data is updated.
// in this function firstly the ranges array is filled, then the min values are 
// computed and saved in specific variables and lastly is called the avoid_walls
// function to avoid the walls of the circuit.
void robotCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

	for(int i = 0; i < 720 ; i++){
		ranges[i] = msg -> ranges[i]; 
	}
	
	float f = min_front_side(ranges);
	float l = min_left_side(ranges);
	float r = min_right_side(ranges);
	
	cout<<"the robot speed is : "<<speed_n<<endl;
	cout<<"min front distance is : "<<f<<endl;
	cout<<"min left distance is : "<<l<<endl;
	cout<<"min right distance is : "<<r<<endl;
	cout<<"--------------------------------------------"<<endl;
	
	avoid_walls(f, l, r);

}

int main(int argc, char ** argv){

	// node initialization 
 	ros::init(argc, argv, "robotController");
	
	// the NodeHandle is used for handling the communication with the ROS system
	// ( it is an object wich rapresents the ROS node ).
	ros::NodeHandle n;
	
	// subscription to the /base_scan topic
	ros::Subscriber sub = n.subscribe("/base_scan",1000,robotCallback);

	// velocity publisher 
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

	// server of the /velocity_control custom service
	ros::ServiceServer service_velocity = n.advertiseService("/velocity_control", velocityControl);

	// it blocks the main thread from exiting until ROS invokes a shutdown.
	ros::spin();

	return 0;
}