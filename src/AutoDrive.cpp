#include <iostream>
#include "AutoDrive.h"


using namespace std;

drive::drive()
{
    ros::NodeHandle nh;

    drive2_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    drive2_sub_Laser = nh.subscribe("scan",100, &drive::laserMsgCallBack, this);
    drive2_sub_Odom = nh.subscribe("odom", 100, &drive::odomMsgCallBack, this);

    CorrectHeading = false;
    FacingDirection = false;
    Tuning = 3;
    Angular_Velocity = 0.0;
    Linear_Velocity = 0.0;
    publishVelocity(2.2, 0.4);

    Goal_X = 0.0;
    Goal_Y = 0.0;

    tempAngle = false;

    Adjusting = Moving = Avoiding = Arrived = Rebound = false;
    
}

drive::~drive(){

}

void drive::laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    if(!SensorReadings.empty()){
        SensorReadings.clear();
    }

    if(!ScanData.empty()){
        ScanData.clear();
    }

    for(int i = 0; i < 360; i += 1){

        SensorReadings.push_back(msg->ranges.at(i));

    }

    if(!ScanData.empty()){
        ScanData.clear();
    }

    for(int i = 269; i < 360; i++){
        
        ScanData.push_back(msg->ranges.at(i));
        
    }

    for(int i = 0; i < 91; i++){
        ScanData.push_back(msg->ranges.at(i));
    }

    Prev_Time = Current_Time;
    Current_Time = msg->header.stamp.sec;
    

}

void drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    Current_X = msg->pose.pose.position.x; // Get the x position
    Current_Y = msg->pose.pose.position.y; // Get the y position

    

    //Retrieve quaternion values
    double quat0 = msg->pose.pose.orientation.w; 
    double quat1 = msg->pose.pose.orientation.y;
    double quat2 = msg->pose.pose.orientation.x;
    double quat3 = msg->pose.pose.orientation.z;

    // Convert quaternion values to euler values
    double Orientation = atan2(2 * (quat0 * quat3 + quat1 * quat2), 1 - 2 * ((quat2 * quat2) + (quat3 * quat3)));

    // Convert the euler radians to degrees
    Orientation = Orientation * (180/M_PI);

    // Re-orientate the angles to a simpler format 
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

    for(int i = 180; i < 0; i-45){
        Bubble_Boundary[i] = (Linear_Velocity * (Current_Time - Prev_Time) * Tuning) + 0.5;
    }


    Current_Theta = Orientation;
    
}


bool drive::adjustHeading(double CurrentXCoordinate, double CurrentYCoordinate, float GoalX, float GoalY, double currentAngle)
{
    //ROS_INFO("Adjusting heading");
    double xDiff = GoalX - CurrentXCoordinate;
    double yDiff = GoalY - CurrentYCoordinate;

    
    double angle = atan2(yDiff,xDiff) * 180/M_PI;

  
    
    if(angle > 0){
        angle = 360 - (abs(angle));
    } 

    if(angle < 0){
        angle = abs(angle);
    }
    //cout << "Target angle: " << angle << endl;

    float difference = ((float)angle - (float)currentAngle + 540);
    difference = fmod(difference,360.f);
    difference -= 180;

    double min = angle - 4;
    double max = angle + 4;

    if(min < 0){min += 360;}
    if(max > 360){max -= 360;}
    //cout << "Current angle: " << currentAngle << endl;

    if(currentAngle < max && currentAngle > min){
        publishVelocity(0.0,0.0);
        FacingDirection = true; 
        return true;
    } else {
        if(difference < 0){
            publishVelocity(0.0,0.2);
            FacingDirection = false;
        } else {
            publishVelocity(0.0,-0.2);
            FacingDirection = false;
        }
        return false;
    }
   
}

void drive::publishVelocity(double Linear, double Angular)
{
    geometry_msgs::Twist msg; // Set message format

    msg.linear.x = Linear; //Set linear velocity
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.z = Angular; //Set Angular velocity
    msg.angular.x = 0;
    msg.angular.y = 0;

    drive2_pub.publish(msg); // Publish message
}

int drive::CheckForObstacles()
{
    //ROS_INFO("Checking for obstacles");
    if(!ScanData.empty()){
        

        for(int i = 180; i >=0; i =- 45){
            cout << "Scan " << i << ": " << ScanData.at(i) << endl;
            if(ScanData.at(i) <= Bubble_Boundary[i]){
                
                return 1;
            }
        }
        return 0;
    }
}

bool drive::CheckForDestination(){
    
    double minimumX = Goal_X - 0.3;
    double maximumX = Goal_X + 0.3;

    double maximumY = Goal_Y + 0.3;
    double minimumY = Goal_Y - 0.3;

    if((Current_X < maximumY) && (Current_X > minimumY) && (Current_Y < maximumX) && (Current_Y > minimumX)){
        ROS_INFO("Destination reached");
        return true;
    } else {
        return false;
    }
}

float drive::ComputeReboundAngle()
{
   

    double top = 0;
    double bottom = 0;

    vector<float> sensorData;
    vector<int> Location;

    if(!ScanData.empty()){

        for(int i = 0; i < 180; i++){
            if(!isinf(ScanData.at(i))){
                sensorData.push_back(ScanData.at(i));
                Location.push_back(i);
            }
        }

        for(int i = 0; i < sensorData.size(); i++){
            top += Location.at(i) * sensorData.at(i);
            bottom += sensorData.at(i);
        }

        float result = top/bottom;
        float min = result - 4;
        float max = result + 4;
        
        cout << "Rebound angle: " << result << endl;

        float difference = (result - (float)Current_Theta + 540);
        difference = fmod(difference,360.f);
        difference -= 180;

        if(Current_Theta > min && Current_Theta < max){
            
            publishVelocity(0.5,0.0);
            Adjusting = false;
        } else {
            if(difference > 0){
                publishVelocity(0.0, 0.5);
                Adjusting = true;
            } else {
                publishVelocity(0.0,-0.5);
                Adjusting = true;
            }
        }
    }

}

bool drive::GoalVisible()
{
    if(!ScanData.empty()){
        double xDiff = Goal_X - Current_X;
        double yDiff = Goal_Y - Current_Y;

        float difference = atan(yDiff/xDiff);
        difference = difference * 180/M_PI;

        difference = round(difference);
        difference = fabs(difference);
       

        double distance = pow(xDiff,2) + pow(yDiff, 2);
        distance = sqrt(distance);
        distance = fabs(distance);
        

        if(ScanData.at(difference) > distance){
            return true;
            //FacingDirection = true;
        } else {
            return false;
            //FacingDirection = false;
        }

    }
}

void drive::MoveForward()
{
    
    if(CheckForObstacles() == 0){
        Linear_Velocity = 0.2;

        if(Angular_Velocity > 0.1){
            publishVelocity(0.0, 0.0);
        } else {
            publishVelocity(Linear_Velocity, 0.0);
            //ROS_INFO("Moving forward");
        }
    } 
    
}

void drive::stopMoving(){
    publishVelocity(0.0, 0.0);
}

bool drive::checkIfMoving()
{
    if(Linear_Velocity < 0.2){
        return false;
    } else {
        return true;
    }
}

bool drive::checkOnTarget(){
    double xDiff = Goal_X - Current_X;
    double yDiff = Goal_Y - Current_Y;

    
    double angle = atan2(yDiff,xDiff) * 180/M_PI;

  
    
    if(angle > 0){
        angle = 360 - (abs(angle));
    } 

    if(angle < 0){
        angle = abs(angle);
    }
    cout << "Angle: " << angle << endl;
    cout << "Current Angle: " << Current_Theta << endl;
    double min = angle - 4;
    double max = angle + 4;

    if(Current_Theta < max && Current_Theta > min){
        return true;
    } else {
        return false;
    }
}

void drive::Debug()
{ 
    
    if(FacingDirection == true){
       if(CheckForObstacles() == 0){
            MoveForward();
       } else {
           ComputeReboundAngle();
       }
    } else {
        adjustHeading(Current_X, Current_Y, Goal_X, Goal_Y, Current_Theta);
    }

    if(CheckForDestination() == true)
    {
        stopMoving();
    }


}

void drive::Sensing()
{
    
   
}

void drive::Control()
{
    /*
    if(!FacingDirection){
        adjustHeading(Current_X, Current_Y, Goal_X, Goal_Y, Current_Theta);
    } else if(Avoiding == false) {
        MoveForward();
    }
    cout << "Obstacles: " << CheckForObstacles() << endl;
    if(CheckForObstacles() == 1){
        Avoiding = true;
        publishVelocity(0.0, 0.5);
    } else if(Avoiding == true){
        MoveForward();
    }

    if(CheckForDestination() == true){
        stopMoving();
    }*/
    cout << "Obstacle detected: " << CheckForObstacles() << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "AutoDrive");
    drive control;
    
    
    ros::Rate loop_rate(60);

    while(ros::ok())
    {
        //control.Debug();
        //control.Sensing();
        control.Control();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}