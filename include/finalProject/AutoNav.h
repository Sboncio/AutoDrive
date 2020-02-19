#ifndef AUTONAV_H_
#define AUTONAV_H_

#include <ros/ros.h>
#include <chrono>
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT 1
#define RIGHT 2

//#define ACCELERATION 0.2

#define GET_DIRECTION 0



class autodrive{
    public:
        autodrive();
        ~autodrive();
        bool init();
        bool controlloop();
        void debug();
       
        

    private:
        //Node Handler
        ros::NodeHandle nh_;

        //ROS topic publisher
        ros::Publisher cmd_vel_pub_;

        //ROS topic subscriber
        ros::Subscriber laser_scan_sub_;
        ros::Subscriber odom_sub_;

        // Variables

        double Linear_velocity_prev;
        double Linear_velocity;
        double Linear_acceleration;
        std::vector<double> Linear_velocity_allowable;

        double Angular_velocity_prev;
        double Angular_velocity;
        double Angular_acceleration;

        double Current_timestamp;
        double Prev_timestamp;
        double Timeslice;

        std::vector<double> Scan_data;

        void laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
        void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg);

        
        void publishVelocity(double Linear, double Angular);

        void loopAllowableVelocities(double velocity, double acceleration, std::vector<double> &AllowableVelocities);

};
#endif