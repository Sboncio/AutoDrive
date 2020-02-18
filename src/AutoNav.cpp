#include "finalProject/AutoNav.h"
#include <iostream>
using namespace std;

//Test command: roslaunch turtlebot3_gazebo turtlebot3_simulation.launch


//Node constructor
autodrive::autodrive()
{
    ROS_INFO("Final Year Project");
    
}

//Node destructor
autodrive::~autodrive()
{
    ros::shutdown(); //Shutdown all publishers 
}

bool autodrive::init()
{
    //Default value
    std::string vel_topic_name = nh_.param<std::string>("vel_topic_name","");
    

    
    //Initialise publishers
    cmd_vel_pub_ = nh_.advertise<nav_msgs::Odometry>("cmd_vel_pub_", 100);

    //Initialise subscriptions
    laser_scan_sub_ = nh_.subscribe("scan",100, &autodrive::laserMsgCallBack, this); // Subscribe to LiDAR
    odom_sub_ = nh_.subscribe("odom", 100, &autodrive::odomMsgCallBack, this); // Subscribe to odometry


    return true;
}

void autodrive::laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    //Get the ranges
    Scan_data[0] = msg->ranges.at(270);
    Scan_data[1] = msg->ranges.at(315);
    Scan_data[2] = msg->ranges.at(0);
    Scan_data[3] = msg->ranges.at(45);
    Scan_data[4] = msg->ranges.at(90);

    for(int i = 0; i < 5; i++){ // ensure there is no infinite ranges
        if(std::isinf(Scan_data[i])){ Scan_data[i] = 100;}
    }
}



void autodrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    Prev_timestamp = Current_timestamp;
    Current_timestamp = msg->header.stamp.nsec;

    Linear_velocity_prev = Linear_velocity;
    Linear_velocity = msg->twist.twist.linear.x;

    Angular_velocity_prev = Angular_velocity;
    Angular_velocity = msg->twist.twist.angular.z;

    Timeslice = Current_timestamp - Prev_timestamp;

    Linear_acceleration = (Linear_velocity - Linear_velocity_prev) / Timeslice;

    Angular_acceleration = (Angular_velocity - Angular_velocity_prev) / Timeslice;

    cout << "Linear acceleration: " << Linear_acceleration << endl;
    cout << "Angular acceleration: " << Angular_acceleration << endl;
}



void autodrive::publishVelocity(double Linear, double Angular)
{
    nav_msgs::Odometry msg; // Set message format

    msg.twist.twist.linear.x = Linear; //Set linear velocity
    msg.twist.twist.angular.z = Angular; //Set Angular velocity

    cmd_vel_pub_.publish(msg); // Publish message
}



bool autodrive::controlloop()
{
 

    return true;
}

void autodrive::debug() // Testing function
{
    
}


int main(int argc, char **argv){

    ros::init(argc, argv, "autodrive_node");

   autodrive controller;
   ros::Rate loop_rate(60);
   controller.init();
   
    while(ros::ok())
    {
        controller.controlloop();
        controller.debug();
     
        ros::spin();
        loop_rate.sleep();
        
    }

   
    return 0;
}