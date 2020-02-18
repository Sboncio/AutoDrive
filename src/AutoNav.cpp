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
    
    //Initialise variables
    escape_range_ = 30.0 * DEG2RAD;
    forward_distance_ = 0.7;
    side_distance_ = 0.6;
    current_pose_ = 0.0;
    previous_pose_ = 0.0;
    Time_Difference = ros::Duration(0);
    current_Time = ros::Time::now();
    previous_Time = ros::Time::now();
    TimeSlice = 0;

    linear_velocity = 0.f;
    angular_velocity = 0.f;

    for(int i = 0; i < 360; i++){scan_data_.push_back(0);}

    
    
    //Initialise publishers
    cmd_vel_pub_ = nh_.advertise<nav_msgs::Odometry>("cmd_vel_pub_", 100);

    //Initialise subscriptions
    laser_scan_sub_ = nh_.subscribe("scan",10, &autodrive::laserMsgCallBack, this); // Subscribe to LiDAR
    odom_sub_ = nh_.subscribe("odom", 100, &autodrive::odomMsgCallBack, this); // Subscribe to odometry


    return true;
}

void autodrive::laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    
    for(int i = 0; i < 360; i++){scan_data_.push_back(msg->ranges.at(i));}


    previous_Time = current_Time; // Set the previous time
    current_Time = ros::Time::now(); // Get the current time
    Time_Difference = current_Time - previous_Time; // Evaluate the time difference
    TimeSlice = current_Time.toSec() - previous_Time.toSec(); 
}

void autodrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    prev_linear_velocity = linear_velocity;
    prev_angular_velocity = angular_velocity;
    linear_velocity = msg->twist.twist.linear.x; //Get the linear velocity
    angular_velocity = msg->twist.twist.angular.z; // Get the angular velocity

    previous_Time = current_Time;
    current_Time = ros::Time::now();
    Time_Difference = current_Time - previous_Time;
    TimeSlice = Time_Difference.toSec();

    linear_acceleration = (linear_velocity - prev_linear_velocity) / TimeSlice;
    angular_acceleration = (angular_velocity - prev_angular_velocity) / TimeSlice;
}

float autodrive::allowableMax(double currentVelocity, double acceleration) // Get the maximum allowable velocity
{
    return (currentVelocity + (acceleration * Time_Difference.toSec()));
}

float autodrive::allowableMin(double currentVelocity, double acceleration) // Get the minimum allowable velocity
{
    return (currentVelocity - (acceleration * Time_Difference.toSec()));
}


float autodrive::findDist(float Linear, float Angular, std::vector<double> scan_data)
{
    // Find the angle
    float theta = (Angular * Time_Difference.toSec());

    if(scan_data_.at(round(theta)) > 1){
        return 100.f;
    } else {
        return scan_data_.at(round(theta));
    }


}

float autodrive::calculateBrakingDistance(float velocity)
{
    
    

}



bool autodrive::controlloop()
{
    int Desired_V = 0.20;

     previous_Time = current_Time;
    current_Time = ros::Time::now();
    Time_Difference = current_Time - previous_Time;
    
    // scan_data_

    // Got allowable velocities
    allowable_V[0] = allowableMin(linear_velocity, linear_acceleration);
    allowable_V[1] = allowableMax(linear_velocity, linear_acceleration);
    allowable_W[0] = allowableMin(angular_velocity, angular_acceleration);
    allowable_W[1] = allowableMax(angular_velocity, angular_acceleration);

    loopAllowableVelocities(allowable_V[0], allowable_V[1], allowableRangeLinear);
    loopAllowableVelocities(allowable_W[0], allowable_W[1], allowableRangeAngular);
    
    // for each v in allowable_V
    for(int i = 0; i <= allowableRangeLinear.size(); i++){
        // for each w in allowable_W
        for(int j = 0; j <= allowableRangeAngular.size(); j++){
            float dist = findDist(allowableRangeLinear.at(i), allowableRangeAngular.at(j), scan_data_);
           /* breakDist = calculateBrakingDistance(v)
            if(dist > breakDist)
                heading = hDiff(robotPose, goalPose, v, w)
                clearance = ((dist - breakDist) / (Dmax - breakDist))
                cost = costFunction(heading, clearance, abs(desired_v - v))
                if(cost > optimal)
                    best_v = v
                    best_w = w
                    optimal = cost*/
                    cout << "Tick" << endl;
        }
    }
    // set robot trajectory to best_v & best_w

return true;
    

}

void autodrive::debug() // Testing function
{
    previous_Time = current_Time;
    current_Time = ros::Time::now();
    Time_Difference = current_Time - previous_Time;
    
    cout << "Minimum Velocity: " << allowable_V[0] << endl;
    cout << "Maximum Velocity: " << allowable_V[1] << endl;

}

void autodrive::loopAllowableVelocities(float min, float max, std::vector<float> &allowable) //Determine all allowable velocities into a vector
{
    for (float i = min; i <= max; i += 0.001)
    {
        allowable.push_back(i);
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
        //controller.debug();
        
        loop_rate.sleep();
    }

   
    return 0;
}