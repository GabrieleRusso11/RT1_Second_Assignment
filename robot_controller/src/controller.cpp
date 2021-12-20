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

float ranges[720], right_side[320], left_side[320], front[80];


//float th_go = 1.0;
float dan = 0.3;
float th_s = 0.4;
float th_min = 0.25;
float th_f = 1.9;
float speed = 5.0;
float th = 2;


ros::Publisher pub;

float min_right_side(float * f){
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

void drive(float speed_l, float speed_a){
	geometry_msgs::Twist robot_velocity;

	
	robot_velocity.linear.x = speed_l;
	robot_velocity.angular.z = speed_a;

	pub.publish(robot_velocity);

}

void turn(float speed){
	geometry_msgs::Twist robot_velocity;

	
	robot_velocity.linear.x = 0;
	robot_velocity.angular.z = speed;

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
		drive(speed,0.0);
	}
	else if(min_front(r) > th && min_left_side(r) < th_s && min_right_side(r) > th_s){
		cout<<"i'm near to a wall on the left"<<endl;
		drive(speed,-speed);
	}
	else if(min_front(r) > th && min_left_side(r) > th_s && min_right_side(r) < th_s){
		cout<<"i'm near to a wall on the right"<<endl;
		drive(speed,speed);
	}
	else if(min_front(r) < th && min_left_side(r) > th_s && min_right_side(r) > th_s){
		if(min_front(r) < dan){
			stop();
			cout<<"danger"<<endl;
			if(min_left_side(r) < min_right_side(r)){
				cout<<"stop and turn right"<<endl;
				turn(-0.8);
				if(front[35] > th_f && front[40] > th_f && front[45] > th_f){
					cout<<"go on"<<endl;
					drive(0.7,0.0);
				}
			}
			else{
				cout<<"stop and turn left"<<endl;
				turn(0.8);
				if(front[35] > th_f && front[40] > th_f && front[45] > th_f){
					cout<<"go on"<<endl;
					drive(0.7,0.0);
				}
			}
		}
		else{
			drive(speed,0.0);
		}
	}
	else if(min_front(r) < th && min_left_side(r) < th_s && min_right_side(r) > th_s){
		if(min_front(r) < dan){
			stop();
			cout<<"danger"<<endl;
			cout<<"stop and turn right"<<endl;
			turn(-1.0);
			if(front[35] > th_f && front[40] > th_f && front[45] > th_f){
				cout<<"go on"<<endl;
				drive(0.7,0.0);
			}
		}
		else{
			cout<<"turn right"<<endl;
			drive(0.7,-1.0);
		}
	}
	else if(min_front(r) < th && min_left_side(r) > th_s && min_right_side(r) < th_s){
		if(min_front(r) < dan){
			stop();
			cout<<"danger"<<endl;
			cout<<"stop and turn left"<<endl;
			turn(1.0);
			if(front[35] > th_f && front[40] > th_f && front[45] > th_f){
				cout<<"go on"<<endl;
				drive(0.7,0.0);
			}
		}
		else{
			cout<<"turn left"<<endl;
			drive(0.7,1.0);
		}
	}
	else if(min_front(r) < th && min_left_side(r) < th_s && min_right_side(r) < th_s){
		if(min_front(r) < dan){
			cout<<"stop ziooo"<<endl;
			stop();
			if(min_left_side(r) < min_right_side(r)){
				cout<<"stop and turn right"<<endl;
				turn(-1.0);
				if(front[35] > th_f && front[40] > th_f && front[45] > th_f){
					cout<<"go on"<<endl;
					drive(0.7,0.0);
				}
			}
			else{
				cout<<"stop and turn left"<<endl;
				turn(1.0);
				if(front[35] > th_f && front[40] > th_f && front[45] > th_f){
					cout<<"go on"<<endl;
					drive(0.7,0.0);
				}
			}
		}
		else{
			if(min_left_side(r) < min_right_side(r)){
				cout<<"turn on right"<<endl;
				drive(0.7,-1.0);
			}
			else{
				cout<<"turn left"<<endl;
				drive(0.7,1.0);
			}
		}
	}
	
}


void robotCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	
	cout<<" th = "<<th<<endl;

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
	cout<<"front right : "<<front[35]<<endl;
	cout<<"front left : "<<front[45]<<endl;
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
