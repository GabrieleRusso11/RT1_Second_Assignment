#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

using namespace std;

float ranges[720], right_side[320], left_side[320], front_side[80];

float dan_th = 0.25;
float th = 1.5;

float speed_n = 2.0;

ros::Publisher pub;

float min_right_side(float * f){
	float min_r = 100;
	int j = 0;
	for(int i = 0; i < 320; i++){
		right_side[j] = f[i];
		if(right_side[j] < min_r){
			min_r = right_side[j];
		}
		j++;
	}
	return min_r;
}

float min_left_side(float * f){
	float min_l = 100;
	int j = 0;
	for(int i = 400; i < 720; i++){
		left_side[j] = f[i];
		if(left_side[j] < min_l){
			min_l = left_side[j];
		}
		j++;
	}
	return min_l;
}

float min_front(float * f){
	float min_f = 100;
	int j=0;
	for(int i = 320; i < 400; i++){
		front_side[j] = f[i];
		if(front_side[j] <  min_f){
			min_f = front_side[j];
		}
		j++;
	}
	return min_f;
}

void drive(float speed_l, float speed_a){

	geometry_msgs::Twist robot_velocity;
	robot_velocity.linear.x = speed_l;
	robot_velocity.angular.z = speed_a;
	pub.publish(robot_velocity);

}

void turn_only(float speed){

	geometry_msgs::Twist robot_velocity;
	robot_velocity.linear.x = 0;
	robot_velocity.angular.z = speed;
	pub.publish(robot_velocity);

}

void avoid_obstacle(float front, float left, float right){

	float speed_r = right;
	float speed_l = left;

	if(front > th){
		drive(speed_n,0.0);
	}
	else if(front < th){
		if(left < right){
			cout<<"turn right"<<endl;
			if(left < dan_th){
				turn_only(-speed_r);
			}
			else{
				drive(speed_r, -1);
			}
		}
		else if(right < left){
			cout<<"turn left"<<endl;
			if(right < dan_th){
				turn_only(speed_l);
			}
			else{
				drive(speed_l, 1);
			}
		}
	}
}

void robotCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

	for(int i = 0; i < 720 ; i++){
		ranges[i] = msg -> ranges[i]; 
	}
	
	float f = min_front(ranges);
	float l = min_left_side(ranges);
	float r = min_right_side(ranges);
	
	cout<<"min front distance = "<<f<<endl;
	cout<<"min left distance = "<<l<<endl;
	cout<<"min right distance = "<<r<<endl;
	cout<<"--------------------------------------------"<<endl;
	
	avoid_obstacle(f, l, r);

}

int main(int argc, char ** argv){

 	ros::init(argc, argv, "robotController");
	ros::NodeHandle n;
	
	ros::Subscriber sub1 = n.subscribe("/base_scan",1000,robotCallback);
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

		
	ros::spin();

	return 0;
}