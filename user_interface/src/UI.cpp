#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <iostream>
#include "robot_controller/Velocity.h"
using namespace std;

int main(int argc, char ** argv){

    // node initialization
    ros::init(argc, argv, "User_interface");

    // the NodeHandle is used for handling the communication with the ROS system
	// ( it is an object wich rapresents the ROS node ).
    ros::NodeHandle n;

    // this creates a client that sends a Reset position request to the /reset_position service
    ros::ServiceClient client_reset = n.serviceClient<std_srvs::Empty>("/reset_positions");

    // this creates a client that sends a Change velocity request to the /velocity_control
    // custom service
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

        // takes the client_velocity request from the user
        cin>>vel.request.command;
        
        // this 'if else if' structure checks what is the user request
        // in order to call the correct serve
        if(vel.request.command == 'r'){
            client_reset.waitForExistence();
            client_reset.call(reset); // client_reset call the reset_position server
        }
        else if(vel.request.command == 'i' || vel.request.command == 'd'){
            client_velocity.waitForExistence();
            client_velocity.call(vel); // client_velocity call the velocity_control server
        }
        else if(vel.request.command == 'q'){
            exit(EXIT_SUCCESS);
        }
        else{
            cout<<"------------------------------------------------"<<endl;
            cout<<"Error!! Please write only the characters displayed."<<endl;
            cout<<"------------------------------------------------"<<endl;
        }

    }

    // it blocks the main thread from exiting until ROS invokes a shutdown.
    ros::spin();

    return 0;
}