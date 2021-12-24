#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <iostream>
#include "robot_controller/Velocity.h"
using namespace std;


char command;

int main(int argc, char ** argv){

    ros::init(argc, argv, "User_interface");
    ros::NodeHandle n;
    ros::ServiceClient client_reset = n.serviceClient<std_srvs::Empty>("/reset_positions");
    ros::ServiceClient client_velocity = n.serviceClient<robot_controller::Velocity>("/velocity_control");

    std_srvs::Empty reset;
    robot_controller::Velocity vel;
    

    while(ros::ok()){
        cout<<endl;
        cout<<"Hi this is the robot user interface, you have the following commands : "<<endl;
        cout<<"-----------------------------------------------------------------------"<<endl;
        cout<<"-- press i if you want to increase the robot velocity."<<endl;
        cout<<"-- press d if you want to decrease the robot velocity."<<endl;
        cout<<"-- press r if you want to reset the robot position. "<<endl;
        cout<<"-- press q if you want to exit."<<endl;
        cout<<"-----------------------------------------------------------------------"<<endl;

        cin>>vel.request.command;
        
        if(vel.request.command == 'r'){
            client_reset.waitForExistence();
            client_reset.call(reset);
        }
        else if(vel.request.command == 'i' || vel.request.command == 'd'){
            client_velocity.waitForExistence();
            client_velocity.call(vel);
        }
        else if(vel.request.command == 'q'){
            exit(EXIT_SUCCESS);
        }
    }
    ros::spin();

    return 0;
}