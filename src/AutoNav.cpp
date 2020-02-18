#include "finalProject/AutoNav.h"
#include <iostream>
using namespace std;

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
    laser_scan_sub_ = nh_.subscribe("scan",10, &autodrive::laserMsgCallBack, this); // Subscribe to LiDAR
    odom_sub_ = nh_.subscribe("odom", 100, &autodrive::odomMsgCallBack, this); // Subscribe to odometry


    return true;
}

void autodrive::laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    
   
}

void autodrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  
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
   ros::Rate loop_rate(10);
   controller.init();
   
    while(ros::ok())
    {
        controller.controlloop();
        //controller.debug();
        
        loop_rate.sleep();
    }

   
    return 0;
}