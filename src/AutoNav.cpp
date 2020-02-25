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
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    //Initialise subscriptions
    laser_scan_sub_ = nh_.subscribe("scan",100, &autodrive::laserMsgCallBack, this); // Subscribe to LiDAR
    odom_sub_ = nh_.subscribe("odom", 100, &autodrive::odomMsgCallBack, this); // Subscribe to odometry


    return true;
}

void autodrive::laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    //Get the ranges
    if(!Scan_data.empty()){
        Scan_data.clear();
    }

    for(int i = 270; i <360; i++){
        Scan_data.push_back(msg->ranges.at(i));
    }

    for(int i = 0; i <90; i++){
        Scan_data.push_back(msg->ranges.at(i));
    }

    for(int i = 0; i < 5; i++){ // ensure there is no infinite ranges
        if(std::isinf(Scan_data.at(i))){ Scan_data.at(i) = 100;}
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

    Position_X = msg->pose.pose.position.x;
    Position_Y = msg->pose.pose.position.y;

    double quat0 = msg->pose.pose.orientation.w;
    double quat1 = msg->pose.pose.orientation.y;
    double quat2 = msg->pose.pose.orientation.x;
    double quat3 = msg->pose.pose.orientation.z;
    Orientation = atan2(2 * (quat0 * quat3 + quat1 * quat2), 1 - 2 * ((quat2 * quat2) + (quat3 * quat3)));
    Orientation = Orientation * (180/M_PI);
    if(Orientation < 0) {
        Orientation += 360.0;
    } 
    if(Orientation > 180){
        Orientation -= 180;
        double temp = 90 - Orientation;
        Orientation = temp + 90;
    } else if (Orientation < 180) {
        double temp = 180 - Orientation;
        Orientation = Orientation + (temp * 2);
    }


    Linear_acceleration = (Linear_velocity - Linear_velocity_prev) / Timeslice;

    Angular_acceleration = (Angular_velocity - Angular_velocity_prev) / Timeslice;

}

void autodrive::publishVelocity(double Linear, double Angular)
{
    geometry_msgs::Twist cmd_vel; // Set message format

    cmd_vel.linear.x = Linear; //Set linear velocity
    cmd_vel.angular.z = Angular; //Set Angular velocity

    cmd_vel_pub_.publish(cmd_vel); // Publish message
}


void autodrive::loopAllowableVelocities(double velocity, double acceleration, std::vector<double> &AllowableVelocities)
{
    if(!AllowableVelocities.empty()){
        AllowableVelocities.clear();
    }

    double minimum = velocity - (acceleration * Timeslice);
    double maximum = velocity + (acceleration * Timeslice);

    for(double i = minimum; i<= maximum; i += 0.001){
        AllowableVelocities.push_back(i);
    }

}

double autodrive::calculateStoppingDistance(double velocity, double acceleration)
{
    double Distance = ((velocity * velocity) / (2 * acceleration));
    return Distance;
}


double autodrive::calculateAngle(double linear, double angular)
{

    float x_pos = Position_X - (linear/angular) * sin(Orientation);
    float y_pos = Position_Y + (linear/angular) * cos(Orientation);

    float temp_Orientation = Orientation + (angular * Timeslice);
    float temp_x_pos = x_pos + (linear/angular) * sin(temp_Orientation);
    float temp_y_pos = y_pos - (linear/angular) * cos(temp_Orientation);

    double rotation = atan2(temp_y_pos - y_pos, temp_x_pos - x_pos);

    cout << rotation << endl;

}

bool autodrive::controlloop()
{
 
    return true;
}

void autodrive::debug() // Testing function
{
    loopAllowableVelocities(Linear_velocity, Linear_acceleration, Linear_velocity_allowable);
    loopAllowableVelocities(Angular_velocity, Angular_acceleration, Angular_velocity_allowable);

    for(int i = 0; i < Linear_velocity_allowable.size(); i++){
        for(int j = 0; j < Angular_velocity_allowable.size(); j++){

            //calculateAngle(Linear_velocity_allowable.at(1), Angular_velocity_allowable.at(j));
            calculateAngle(Linear_velocity_allowable.at(i),Angular_velocity_allowable.at(j));
        }
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "autodrive_node");

    autodrive controller;
    ros::Rate loop_rate(10);
    controller.init();
   
    while(ros::ok())
    {
        controller.controlloop();
        controller.debug();
        

        loop_rate.sleep();
        ros::spinOnce();
        
        
    }

   
    return 0;
}