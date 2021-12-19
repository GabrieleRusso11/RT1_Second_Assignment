#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

using namespace std;

float min_l_f = 100;
float min_r_f = 100;
float min_f = 100;
float min_r = 100;
float min_l = 100;
float ranges[720], right_side[240], left_side[240];
float left_front[80], right_front[80], front[80];
float th = 2.0;
float th_go = 1.0;
float dan = 0.5;
float th_s = 0.6;
float th_f = 1.9;


ros::Publisher pub;

float min_right_side(float * f){
	int j = 0;
	for(int i = 0; i < 240; i++){
		right_side[j] = f[i];
		if(right_side[j] < min_r){
			min_r = right_side[j];
		}
		j++;
	}
	return min_r;
}

float min_left_side(float * f){
	int j = 0;
	for(int i = 480; i < 720; i++){
		left_side[j] = f[i];
		if(left_side[j] < min_l){
			min_l = left_side[j];
		}
		j++;
	}
	return min_l;
}

float min_front(float * f){

	int j=0;
	for(int i = 320; i < 400; i++){
		front[j] = f[i];
		if(front[j] <  min_f){
			min_f = front[j];
		}
		j++;
	}
	return min_f;

}

float min_left_front(float * f){

	int j=0;
	for(int i = 400; i < 480; i++){

		left_front[j] = f[i];
		//cout<<"left"<<left_front[j]<<endl;
		if(left_front[j] < min_l_f){
			min_l_f = left_front[j];
		}
		j++;
	}
	return min_l_f;

}

float min_right_front(float * f){

	int j=0;
	for(int i = 240; i < 320; i++){
		right_front[j] = f[i];
		//cout<<"right"<<right_front[j]<<endl;
		if(right_front[j] < min_r_f){
			min_r_f = right_front[j];
		}
		j++;
	}
	return min_r_f;
}



void drive(float speed_l, float speed_a){
	geometry_msgs::Twist robot_velocity;

	
	robot_velocity.linear.x = speed_l;
	robot_velocity.angular.z = speed_a;

	pub.publish(robot_velocity);

}

void stop(){
	geometry_msgs::Twist robot_velocity;

	
	robot_velocity.linear.x = 0;
	robot_velocity.angular.z = 0;

	pub.publish(robot_velocity);

}


void avoid_obstacle(float * r){
	if(min_front(r) > th && min_left_side(r) > th_s && min_right_side(r) > th_s){
		drive(10.0,0.0);
	}
	else if(min_front(r) < th_go){
		if(min_left_side(r) < min_right_side(r) && min_left_side(r) < 0.4){
			cout<<"turn right"<<endl;
			drive(0.8,0.9);
			
		}
		else if(min_right_side(r) < min_left_side(r) && min_right_side(r) < 0.4){
			cout<<"turn left"<<endl;
			drive(0.8,-0.9);
		}
		else{
			cout<<"turn left"<<endl;
			drive(0.8,-0.9);
		}
	}
	else if(min_front(r) < dan){
		if(min_left_side(r) < min_right_side(r)){
			cout<<"turn right"<<endl;
			drive(0.0,0.9);
			if(front[40] > th_f){
				cout<<"go on"<<endl;
				drive(0.7,0.0);
			}
		}
		else if(min_right_side(r) < min_left_side(r)){
			cout<<"turn left"<<endl;
			drive(0.0,-0.9);
			if(front[40] > th_f){
				cout<<"go on"<<endl;
				drive(0.7,0.0);
			}
		}
		else{
			drive(0.0,-0.9);
			if(front[40] > th_f){
				cout<<"go on"<<endl;
				drive(0.7,0.0);
			}
		}
	}
	
	/*else if(th_go < min_front(r) < th && min_left_side(r) > th_s && min_right_side(r) > th_s){
		drive(0.8,0.0);
		cout<<"ciao"<<endl;
	}	*/
	else if(min_front(r) > th && min_left_side(r) > th_s && min_right_side(r) < th_s){
		cout<<"turn left"<<endl;
		drive(0.0,0.9);
	}
	else if(min_front(r) > th && min_left_side(r) < th_s && min_right_side(r) > th_s){
		cout<<"turn right"<<endl;
		drive(0.0,-0.9);
	}
	else{
		drive(0.7,0.0);
		cout<<"go on slowly"<<endl;
	}
	
	/*if(min_front(r) > th && min_left_front(r) > th && min_right_front(r) > th){
		drive(2.0,0.0);
	}
	else if(min_front(r) < th && min_left_front(r) > th && min_right_front(r) > th){
		if(min_left_side(r) < th_s && min_right_side(r) > th_s){
			drive(0.0,0.4);
		}
		else if(min_left_side(r) > th_s && min_right_side(r) < th_s){
			drive(0.0,-0.4);
		}
		else{
			drive(0.0,0.4);
		}
	}
	else if(min_front(r) > th && min_left_front(r) > th && min_right_front(r) < th){
		drive(0.0,-0.4);
	}
	else if(min_front(r) > th && min_left_front(r) < th && min_right_front(r) > th){
		drive(0.0,0.4);
	}
	else if(min_front(r) < th && min_left_front(r) > th && min_right_front(r) < th){
		drive(0.0,-0.4);
	}
	else if(min_front(r) < th && min_left_front(r) < th && min_right_front(r) > th){
		drive(0.0,0.4);
	}
	else if(min_front(r) < th && min_left_front(r) < th && min_right_front(r) < th){
		if(min_left_side(r) < th_s && min_right_side(r) > th_s){
			drive(0.0,0.4);
		}
		else if(min_left_side(r) > th_s && min_right_side(r) < th_s){
			drive(0.0,-0.4);
		}
		else{
			drive(0.0,0.4);
		}
	}
	else if(min_front(r) > th && min_left_front(r) < th && min_right_front(r) < th){
		drive(0.8,0.0);
	}*/
}

void robotCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	
	for(int i = 0; i < 720 ; i++){
		ranges[i] = msg -> ranges[i]; 
		//cout<< "the front range is : " << ranges[i] << endl;
	}
	
	//cout<<"min left front side : "<< min_left_front(ranges) <<endl;
	//cout<<"min right front side : "<< min_right_front(ranges) <<endl;
	cout<<"min front side : "<< min_front(ranges) <<endl;
	cout<<"min left side : "<<min_left_side(ranges) <<endl;
	cout<<"min_right_side : "<< min_right_side(ranges) <<endl;
	cout<<"front : "<<front[40]<<endl;
	cout<<"--------------------------------------------"<<endl;
	
	avoid_obstacle(ranges);
}

int main(int argc, char ** argv){

 	ros::init(argc, argv, "robotController");
	ros::NodeHandle n;
	
	ros::Subscriber sub1 = n.subscribe("/base_scan",1000,robotCallback);
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

		
	ros::spin();

	return 0;
}
