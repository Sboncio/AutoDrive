#ifndef AUTODRIVE_H_
#define AUTODRIVE_H_

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <cmath>
#include <time.h>
#include <string>

#include <fstream>

#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

using namespace std;

/*!
    \class drive
    \brief The class containing the control systems variables and function

    
*/

class drive {
    public:
        drive();
        ~drive();
        void Init();
        void Control();
        void Debug();

    private:
        ros::Publisher drive2_pub; //!< Create a publisher to set velocities
        ros::Publisher marker_pub; //!< Create the publisher to plot the path
        ros::Publisher state_pub;

        ros::Subscriber drive2_sub_Laser; //!< Subscriber to read LiDAR data
        ros::Subscriber drive2_sub_Odom; //!< Subscriber to read odometry data
        ros::Subscriber drive2_interface;

        int degreeOfSeparation = 18; //!< degrees between lidar ranges
        std::vector<float> SensorReadings; //!< Pure sensor data
        std::vector<float> ScanData; //!< Filtered scan data

        float Bubble_Boundary[181]; //!< The boundaries for the bubble rebound algorithm
        bool moving;
        bool start_trigger;

        float Current_X_; //!< Current X coordinate of the system.
        float Current_Y_; //!< Current Y coordinate of the system.
        double Current_Theta_; //!< Current angle that the system is facing
        double Target; //!< Target angle to reach

        float Goal_X_; //!< Goal X coordinate
        float Goal_Y; //!< Goal Y coordinate

        float Avoid_X; //!< X coordinate of detected obstacle
        float Avoid_Y; //!< Y coordinate of detected obstacle

        double Linear_Velocity; //!< Current linear velocity of the system
        double Angular_Velocity; //!< Current angular velocity of the system

        float Prev_Time = 0; //!< Previous time a package was received
        float Current_Time = Prev_Time; //!< Current time

        float Tuning; //!< Tuning parameter for bubble rebound algorithm

        double target_angle; //!< Locked angle for adjustments

        // Flags
        bool CorrectHeading; //!< System is facing towards destination
        bool FacingDirection; //!< Facing necessary angle

        bool tempAngle; //!< Temporary angle for avoidance

        bool Adjusting; //!< System is currently adjusting
        bool Moving; //!< System is currently moving while adjusting
        bool Avoiding; //!< System is currently avoiding an obstacle
        bool Arrived; //!< System has arrived at the destination
        bool Rebound; //!< System is moving towards an area clear of obstacles

        bool AngleLocked; //!< Lock the angle

        bool Finalise;
        
        int map_count; //!< Keep track of map points

        string current_state;

        

        ros::Time Start;
        ros::Duration length;
        
        void laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
        void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg);
        void interfaceCallBack(const geometry_msgs::Vector3::ConstPtr& msg);

        bool adjustHeading(double CurrentXCoordinate, double CurrentYCoordinate, float GoalX, float GoalY, double currentAngle);

        void publishVelocity(double Linear, double Angular);
        void publishState(string state);

        int CheckForObstacles();

        bool CheckForDestination();

        float ComputeReboundAngle();

        void AdjustAngle(double TargetAngle);

        bool GoalVisible();

        void MoveForward();

        void stopMoving();

        bool checkIfMoving();

        bool checkOnTarget();

        void shove();

        void plotPath();

};



#endif