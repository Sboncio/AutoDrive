#ifndef AUTODRIVE_H_
#define AUTODRIVE_H_

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <cmath>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class drive {
    public:
        drive();
        ~drive();
        void Init();
        void Control();
        void Debug();
        void Sensing();

    private:
        ros::Publisher drive2_pub;
        ros::Subscriber drive2_sub_Laser;
        ros::Subscriber drive2_sub_Odom;

        int degreeOfSeparation = 18;
        std::vector<float> SensorReadings;
        std::vector<float> ScanData;

        float Bubble_Boundary[90];
        bool moving;

        float Current_X;
        float Current_Y;
        double Current_Theta;
        double Target;

        float Goal_X;
        float Goal_Y;

        double Linear_Velocity;
        double Angular_Velocity;

        float Prev_Time = 0;
        float Current_Time = Prev_Time;

        float Tuning;

        // Flags
        bool CorrectHeading;
        bool FacingDirection;

        bool tempAngle;

        bool Adjusting;
        bool Moving;
        bool Avoiding;
        bool Arrived;
        bool Rebound;

        
        void laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
        void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg);

        bool adjustHeading(double CurrentXCoordinate, double CurrentYCoordinate, float GoalX, float GoalY, double currentAngle);

        void publishVelocity(double Linear, double Angular);

        int CheckForObstacles();

        bool CheckForDestination();

        float ComputeReboundAngle();

        bool GoalVisible();

        void MoveForward();

        void stopMoving();

        bool checkIfMoving();

        bool checkOnTarget();

        

        

};



#endif