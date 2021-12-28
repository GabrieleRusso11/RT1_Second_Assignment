# Gabriele Russo's second assignment for the Research Track 1 course (Mat. 5180813)

## Introduction 
The aim of this project is similar to the first one, but in this case the robot, autonomously, has to run across this race track, avoiding the walls :

![tracciato](https://github.com/GabrieleRusso11/RT1_Second_Assignment/blob/main/second_assignment/world/tracciato.png)

## Installation and how to run
Firstly download in the src folder of your ROS workspace, this repository using the following command in the shell :

```
git clone https://github.com/GabrieleRusso11/RT1_Second_Assignment.git
```
Secondly come back to the root folder of your workspace and run : 

```
catkin_make
rospack profile
```
Lastly open four shells. In the first shell run :

```
roscore
```
In the second one open the simulation environment :

```
rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```
In the third one run the robot controller node :

```
rosrun robot_controller robotController_node
```
And in the last one run the User Interface node :

```
rosrun user_interface user_interface_node
```

## How it works
For the implementation of this project is used ROS (Robot Operating System).
ROS works through nodes that communicate with each other using Topic (which is messages transport system that implement the publish/subscribe model) and Service (which implement the request/response model).

In this project there are two nodes :

* The Robot Controller node

* The User Interface node

### Robot Controller node
This node, is the main node of the project, it controlls the motion of the robot acquiring data from the Laser Scan Sensor.
It uses two topics for controlling the robot in the stageros node environment :

* /cmd_vel

* /base_scan

Both are stageros node topics, the first one is used by the robot controller node to publish the linear and the angular speed of the robot ,whereas the second one is a topic that contains the robot laser scan sensor data, so the robot controller node subscibe to this topic to see the distance of the robot from the walls.
The laser scan detect data from 0 to 180 degrees and put this data in an array of 720 elements, called ranges. 
The robot controller node split this array in three parts : 

* the right side part, that takes data of ranges array from 0 to 320.
It is used to detect walls/obstacle in the left side of the robot.

* the front side part, that takes data of ranges array from 320 to 400.
it is used to detect walls/obstacle in the frontal side of the robot.

* the left side part, that takes data of ranges array from 400 to 720.
It is used to detect walls/obstacle in the right side of the robot.

To see if the wall is close to the robot in one or more of this three side, the robot controller node finds the minimum value among the data of each side, through this three functions :

``` c++
float min_left_side();

float min_front_side();

float min_right_side();
```

This three functions return the minimum value of each side, and then they are sent as input parameters to this function :

``` c++
void avoid_walls(float front, float left, float right);
```

The `avoid_walls()` function checks firstly if the frontal minimum value is greater or less than the threshold, so if the robot is or isn't close to a wall ahead.
If the frontal minimum value is greater than the threshold, then the wall ahead is still far and so the robot go straight with a linear speed different from zero, whearas the robot is close to a wall, so it checks if turn right or left.
If the minimum value of the left side is less than the minimum value of the right side, the robot has to turn right (because on its left there is a wall), if the the minimum value of the left side is less than the dangerous threshold, the robot is too close to the wall, so it has turn only with an angular speed different from zero, otherwise the robot turns right with an angular and linear speed different to zero.
It is the same if the the minimum value of the right side is less than the minimum value of the left side, but in this case, obviously, the robot has to turn left.
The previous functions are all called in the `robotCallback()` function which is a function that is called each time a message is published on the /base_scan topic, so every time the laser scan data is updated.

As said before, to move the robot the robot controller node has to publish the linear and the angular speed on the /cmd_vel topic, to do this the robot controller node use the following functions :

``` c++
void drive(float speed_l, float speed_a);

void turn_only(float speed_a);
```

Lastly the robot controller node implement a custom service to increase or decrease the robot speed through an user request taken by the user interface node.
To implement this service the robot controller node use the following function :

```c++
bool velocityControl(robot_controller::Velocity::Request &req, robot_controller::Velocity::Response &res);
```

This is a function that is called when a client sends a request to the custom service /velocity_control that takes as request a command by the user and respond with an increment or a decrement of the robot speed.

Below is shown the robot controller pseudocode :

```
While the simulation is running

    listen for User Interface node

    if User Interface sends a request (command)

        if the request is equal to 'i'

            increase the speed (response) of 0.5

        else if the request is equal to 'd'

            decrease the speed (response) of 0.5

    take sensor data from the laser scan and save 
    each data in the ranges array (size = 720)

    Save ranges array data from 0 to 320 in right_side array
    and evaluate the minimum value.

    Save ranges array data from 320 to 400 in front_side array
    and evaluate the minimum value.

    Save ranges array data from 400 to 720 in left_side array
    and evaluate the minimum value.

    if the minimum value of front_side is greater than the threshold

        drive straight with the linear speed 
        increased or decreased by the user

    else if the minimum value of front_side is less than the threshold

        if the minimum value of left_side is less than the minimum value of right_side

            if the minimum value of left_side is less than the dangerous threshold (too close to the wall)

                turn right with a linear speed equals to zero and with and
                angular speed equals to the sum of the right and left distance from the walls

            else 

                turn right with a linear speed equals to the sum of the right and 
                left distance from the walls and an angular speed equals to -1.7

        else if the minimum value of right_side is less than the minimum value of the left_side 

            if the minimum value of right_side is less than the dangerous threshold (too close to the wall)

                turn left with a linear speed equals to zero and with an
                angular speed equals to the sum of the right and left distance from the walls

            else

                turn left with a linear speed equals to the sum of the right and 
                left distance from the walls and an angular speed equals to -1.7
```

### User Interface Node
This node is used to allow the user to reset the robot position in the simulation environment, and to increase or decrease the robot speed.

To reset the robot position this node uses, as client, the stageros node service called `reset_position`, whereas to increase or decrease the robot speed the user interface node uses, as client, the robot controller custom service `velocity_control`.

So it takes from the user a command as request, this command can be :

* 'r' to reset the robot position

* 'i' to increase the robot speed

* 'd' to decrease the robot speed

* 'q' to exit

Below is shown the user interface pseudocode : 

```
while the simulation is running 

    listen for user request (command)

    if the user request is 'r'

        reset the robot position

    else if the user request is 'i' or 'd'

        call the server that manage the custom service
        for increasing or decreasing the robot speed

    else if the user request is 'q'

        exit

    else

        user typing error
```