#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <iostream>
using namespace std;


char command;

int main(int argc, char ** argv){

    ros::init(argc, argv, "User_interface");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/reset_positions");
    std_srvs::Empty reset;

    

    while(ros::ok()){
        cout<<"press r if you want to reset the robot position : "<<endl;
        cin>>command;
        
        if(command == 'r'){
            client.waitForExistence();
            client.call(reset);
        }
    }
    ros::spin();

    return 0;
}