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
        void loopAllowableVelocities(float min, float max, std::vector<float> &allowable);

        float getAccel();
        float calculateBrakingDistance(float velocity);
    private:
        //Node Handler
        ros::NodeHandle nh_;

        //ROS topic publisher
        ros::Publisher cmd_vel_pub_;

        //ROS topic subscriber
        ros::Subscriber laser_scan_sub_;
        ros::Subscriber odom_sub_;

        // Variables
        float linear_velocity = 0.0;
        float prev_linear_velocity;
        float angular_velocity = 0.0;
        float prev_angular_velocity;

        double escape_range_;
        double forward_distance_;
        double side_distance_;

        std::vector<double> scan_data_; //-90, -67, -45, -22, 0, 22, 45, 67, 90
        

        float linear_acceleration = 0.1;
        float angular_acceleration = 0.1;

        double current_pose_;
        double previous_pose_;

        float allowable_V[2];
        float allowable_W[2];
        std::vector<float> allowableRangeLinear;
        std::vector<float> allowableRangeAngular;

        ros::Duration Time_Difference; //Used to compute max/min velocity

        ros::Time current_Time;
        ros::Time previous_Time;

        double TimeSlice;

        void laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
        void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg);
    

        float allowableMax(double currentVelocity, double acceleration);
        float allowableMin(double currentVelocity, double acceleration);  

        float findDist(float Linear, float Angular, std::vector<double> scan_data);      
};
#endif