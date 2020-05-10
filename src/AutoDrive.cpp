#include <iostream>
#include "AutoDrive.h"


using namespace std;


/*! \fn drive::drive()
*   \brief The constructor for the drive class.
*   Called when the object is created.
*/
drive::drive()
{
    ros::NodeHandle nh; //!< Create a node handler

    drive2_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); //!< Set the publisher 
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10); //!< Set the publisher to plot path

    drive2_sub_Laser = nh.subscribe("scan",100, &drive::laserMsgCallBack, this); //!< Subscribe to LiDAR
    drive2_sub_Odom = nh.subscribe("odom", 100, &drive::odomMsgCallBack, this); //!< Subscribe to odometry


    //! Set flags
    CorrectHeading = false;
    FacingDirection = false;
    Tuning = 0.3;
    Angular_Velocity = 0.0;
    Linear_Velocity = 0.0;
    publishVelocity(2.2, 0.4); //!< Set initial velocity

    Goal_X = 0.0; //!< Set the goal X coordinate
    Goal_Y = 0.0; //!< Set the goal Y coordinate
    target_angle = 0; 

    map_count = 0; //!< Variable for ID of path plotting

    tempAngle = false;

    Adjusting = Moving = Avoiding = Arrived = Rebound = AngleLocked = Finalise =  false;
    
   
    Start = ros::Time::now();
}

/*!
    \fn drive::~drive()
    \brief The destructor of the drive class
*/
drive::~drive(){

}

/*!
    \fn void drive::laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
    \brief Function which is called when data is detected from the LiDAR sensor.

    \param msg The message received from the ROS master containing data from the LiDAR
*/
void drive::laserMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    if(!SensorReadings.empty()){ //!< Clear vector if needed
        SensorReadings.clear();
    }

    if(!ScanData.empty()){ //!< Clear vector if needed
        ScanData.clear();
    }

    for(int i = 359; i >= 0; i -= 1){

        SensorReadings.push_back(msg->ranges.at(i)); //!< Store raw data

    }


    int MAX_RANGE = 10;

    
    //! Store filtered data
    for(int i = 269; i < 360; i++){
        if(!isinf(msg->ranges.at(i))){
            ScanData.push_back(msg->ranges.at(i));
        } else {
            ScanData.push_back(MAX_RANGE);
        }
    }

    for(int i = 0; i < 91; i++){
        if(!isinf(msg->ranges.at(i))){
            ScanData.push_back(msg->ranges.at(i));
        } else {
            ScanData.push_back(MAX_RANGE);
        }
    }
    

    Prev_Time = Current_Time; //!< Set previous time variable
    Current_Time = msg->header.stamp.sec; //!< Set current time variable
    

}

/*!
    \fn void drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg)
    \brief Function that is called when odometry data is available
*/

void drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    Current_X = msg->pose.pose.position.x; //!< Get the x position
    Current_Y = msg->pose.pose.position.y; //!< Get the y position

    

    //! Retrieve quaternion values
    double quat0 = msg->pose.pose.orientation.w; 
    double quat1 = msg->pose.pose.orientation.y;
    double quat2 = msg->pose.pose.orientation.x;
    double quat3 = msg->pose.pose.orientation.z;

    //! Convert quaternion values to euler values
    double Orientation = atan2(2 * (quat0 * quat3 + quat1 * quat2), 1 - 2 * ((quat2 * quat2) + (quat3 * quat3)));

    //! Convert the euler radians to degrees
    Orientation = Orientation * (180/M_PI);

    //! Re-orientate the angles to a simpler format 
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

    for(int i = 0; i <= 180; i++){
        Bubble_Boundary[i] = Linear_Velocity * (Current_Time - Prev_Time) + Tuning; //!< Set the bubble boundary
    }


    Current_Theta = Orientation; //!< Save the current theta
    
}

/*!
    \fn bool drive::adjustHeading(double CurrentXCoordinate, double CurrentYCoordinate, float GoalX, float GoalY, double currentAngle)
    \brief Adjusts the turtlebot3 to the heading to face the destination.

    \param CurrentXCoordinate The current X position of the robot relative to the world
    \param CurrentYCoordinate The current Y position of the robot relative to the world
    \param GoalX The X coordinate of the destination
    \param GoalY The Y coordinate of the destination
    \param currentAngle The current angle of the robot in degrees

    
*/
bool drive::adjustHeading(double CurrentXCoordinate, double CurrentYCoordinate, float GoalX, float GoalY, double currentAngle)
{
    
    ROS_INFO("Adjusting heading");
    
    double xDiff = GoalX - CurrentXCoordinate; //!< Obtain difference in X coordinates
    double yDiff = GoalY - CurrentYCoordinate; //!< Obtain difference in Y coordinates

    
    double angle = atan2(yDiff,xDiff) * 180/M_PI; //!< Convert angle to degrees

  
    //! Normalise angle
    if(angle > 0){
        angle = 360 - (abs(angle));
    } 

    if(angle < 0){
        angle = abs(angle);
    }

    //! Determine if robot should turn left or right
    float difference = ((float)angle - (float)currentAngle + 540);
    difference = fmod(difference,360.f);
    difference -= 180;

    double min = angle - 4; //!< Set minimum boundary
    double max = angle + 4; //!< Set maximum boundary

    if(min < 0){min += 360;}
    if(max > 360){max -= 360;}
    

    //!< Move to appropriate heading
    if(currentAngle < max && currentAngle > min){
        publishVelocity(0.5,0.0);
        FacingDirection = true; 
        
        return true;
    } else {
        if(difference < 0){
            publishVelocity(0.0,0.3);
            FacingDirection = false;
        } else {
            publishVelocity(0.0,-0.3);
            FacingDirection = false;
        }
        return false;
    }
   
}

/*!
    \fn void drive::publishVelocity(double Linear, double Angular)
    Function to publish the velocity, setting linear and angular velocity of the robot.

    \param Linear The desired linear velocity of the robot
    \param Angular The desired angular velocity of the robot
*/

void drive::publishVelocity(double Linear, double Angular)
{
    geometry_msgs::Twist msg; //!< Set message format

    msg.linear.x = Linear; //!< Set linear velocity
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.z = Angular; //!< Set Angular velocity
    msg.angular.x = 0;
    msg.angular.y = 0;

    drive2_pub.publish(msg); //!< Publish message
}

/*! 
    \fn int drive::CheckForObstacles()
    \brief Function to check if there are any obstacles detected
*/
int drive::CheckForObstacles()
{
    ROS_INFO("Checking for obstacles");
    if(!ScanData.empty()){ //!< Ensure vector is populated
        

        for(int i = 70; i <= 110; i += 10){ 
            
            if(ScanData.at(i+1) <= Bubble_Boundary[i+1]){ //!< See if obstacle is within range
                ROS_INFO("Obstacle detected");
                return 1;
            }
        }
        return 0;
    }
}
/*!
    \fn bool drive::CheckForDestination()
    \brief Function to check whether the robot has reached the destination set
*/
bool drive::CheckForDestination(){
    
    //! Create boundary
    double minimumX = Goal_X - 0.5; 
    double maximumX = Goal_X + 0.5;

    double maximumY = Goal_Y + 0.5;
    double minimumY = Goal_Y - 0.5;

    //! Check if robot is within goal boundary
    if((Current_X < maximumY) && (Current_X > minimumY) && (Current_Y < maximumX) && (Current_Y > minimumX)){
        ROS_INFO("Destination reached");
        if(!Finalise){
            length = ros::Time::now() - Start; //!< Calculate duration of journey
            Finalise = true; //!< Lock the duration
        }
        cout << "Length: " << length << endl;
        return true;
    } else {
        return false;
    }
}

/*!
    \fn float drive::ComputeReboundAngle()
    \brief Computes the rebound angle necessary for the Bubble Rebound Algorithm
*/
float drive::ComputeReboundAngle()
{
    ROS_INFO("Computing rebound angle");
    double top = 0;
    double bottom = 0;
    float result = 0;

    vector<float> sensorData;
    vector<int> Location;

    if(!SensorReadings.empty()){ //!< Ensure vector is populated

        
        float top = 0;
        float bottom = 0;

        for(int i = 270; i >= 0; i -=45){ //!< Perform a sweep
            top += i * SensorReadings.at(i);
            bottom += SensorReadings.at(i);
        }

        result = top/bottom; //!< Calculate rebound angle

        if(isnan(result)){
            

        result = Current_Theta; //!< Calculate rebound angle
        }

        cout << "Result: " << result << endl;
        target_angle = result;
        return result;
    }
}

/*!
    \fn void drive::AdjustAngle(float TargetAngle)
    \brief Adjusts the robots angle to align with the rebound angle

    \param TargetAngle The angle that was calculated
*/
void drive::AdjustAngle(double TargetAngle)      
{
    ROS_INFO("Adjusting angle");
    float max = TargetAngle + 3; //!< Set maximum boundary
    float min = TargetAngle - 3; //!< Set minimum boundary
    
    //! Calculate if system should turn left or right
    float difference = (TargetAngle - (float)Current_Theta + 540);
    difference = fmod(difference,360.f);
    difference -= 180;
    cout << "Target angle: " << TargetAngle << endl;
    
    //! Check if the system is facing the right direction
    if(Current_Theta > min && Current_Theta < max){
        
        publishVelocity(0.0,0.0);
        Avoid_X = Current_X;
        Avoid_Y = Current_Y;
        AngleLocked = false;
        Rebound = false;
        Adjusting = true;
    } else {
        if(difference < 0){
            publishVelocity(0.0, 0.3);
        } else {
            publishVelocity(0.0,-0.3);
        }
    }
    
}

/*!
    \fn bool drive::GoalVisible()
    \brief Uses the pythagorean theorem to detect if there is an obstacle between the robot and the destination
*/
bool drive::GoalVisible()
{
    ROS_INFO("Checking of goal is visible");
    if(!SensorReadings.empty()){
        double xDiff = Goal_X - Current_X; //!< Calculate x difference
        double yDiff = Goal_Y - Current_Y; //!< Calculate y difference

        float angle= atan2(yDiff,xDiff); //!< Calculate angle to goal
        angle *= 180/M_PI; //!< Convert degrees to radians

        //! Normalise angles
        if(angle > 0){
            angle = 360 - (abs(angle));
        } 

        if(angle < 0){
            angle = abs(angle);
        }
        
        float Adjustment_Angle = 360 - Current_Theta;
        float Relative_Angle = angle + Adjustment_Angle;
        Relative_Angle = round(Relative_Angle);

        if(Relative_Angle >=360){Relative_Angle -= 360;}
        

        //! Get distance to goal (Pythagoras)
        double hypo = pow(xDiff, 2) + pow(yDiff,2);
        hypo = sqrt(hypo); //!< Calculate distance to goal

        if(hypo - SensorReadings.at(Relative_Angle) < 0.2){ //!< Check if goal can be seen
            return true;
        } else {
            return false;
        }
       

    }
}

/*!
    \fn void drive::MoveForward()
    \brief Moves the robot forward at a speed of 0.2
*/
void drive::MoveForward()
{
    ROS_INFO("Moving forward");
    if(CheckForObstacles() == 0){
        Linear_Velocity = 0.2;

        if(Angular_Velocity > 0.1){
            publishVelocity(0.0, 0.0);
        } else {
            publishVelocity(Linear_Velocity, 0.0);
        }
    } 
    
}
/*!
    \fn void drive::stopMoving()
    \brief Stop the system

    Set the systems angular and linear velocities to 0.0
*/
void drive::stopMoving(){
    ROS_INFO("Stopping movement");
    publishVelocity(0.0, 0.0);
}

/*!
    \fn bool drive::checkIfMoving()
    \brief Return if the robot is in motion
*/
bool drive::checkIfMoving()
{
    if(Linear_Velocity < 0.1){
        return false;
    } else {
        return true;
    }
}

/*!
    \fn bool drive::checkOnTarget()
    \brief Checks if the system orientation is correct to reach the destination
*/
bool drive::checkOnTarget(){
    ROS_INFO("Checking if system is on target");
    double xDiff = Goal_X - Current_X;
    double yDiff = Goal_Y - Current_Y;
    
    double angle = atan2(yDiff,xDiff) * 180/M_PI;

  
    
    if(angle > 0){
        angle = 360 - (abs(angle));
    } 

    if(angle < 0){
        angle = abs(angle);
    }

    double min = angle - 4;
    double max = angle + 4;

    if(Current_Theta < max && Current_Theta > min){
        return true;
    } else {
        return false;
    }
}

/*!
    \fn void bool::shove()
    \brief Move the system slightly to avoid obstacles
*/
void drive::shove()
{
    ROS_INFO("Moving forward");
    float xDiff = fabs(Current_X - Avoid_X);
    float yDiff = fabs(Current_Y - Avoid_Y);
    

    if(yDiff < 0.05 || xDiff < 0.05){
        MoveForward();
    } else {
        Adjusting = false;
    }
}

void drive::plotPath()
{
    visualization_msgs::Marker points; //!< Create object for points and object for lines
    points.header.frame_id   = "/map"; //!< Set the frame_ID to "map"
    points.header.stamp  = ros::Time::now(); //!< Set the time header to the current time
    points.ns =  "path"; //!< Set the namespace of the objects
    points.action = visualization_msgs::Marker::ADD; //!< Ensure new shapes are added, not replaced

    points.id = map_count;
    map_count++;

    points.type = visualization_msgs::Marker::POINTS; //!< Set the object type
    points.scale.x = 0.25; //!< Set size of the point
    points.scale.y = 0.25; //!< Set size of the point

    //! Set the colour
    points.color.r = 1.0f;
    points.color.g = 0.0f;
    points.color.b = 1.0f;
    points.color.a = 1.0f;

    float xPos = Current_X;
    float yPos = Current_Y;
    
    geometry_msgs::Point p1;

    //! Set the x and y positions
    p1.x = xPos;
    p1.y = yPos;
    p1.z = 1;

    points.points.push_back(p1);

    marker_pub.publish(points); //!< Publish the points to be shown
}



/*!
    \fn void drive::Debug()
    \brief Function used for testing
*/
void drive::Debug()
{ 
  
}

/*!
*    \fn void drive::Control()
*    \brief The function containing the control algorithm
*/
void drive::Control()
{
    if(CheckForObstacles() == 0 && Rebound == false){ //!< Ensure that there are no obstacles

        if(Adjusting){
            shove();
        } else {
        
            if(!FacingDirection){
                adjustHeading(Current_X, Current_Y, Goal_X, Goal_Y, Current_Theta);
            } else if(Avoiding == false) {
                if(GoalVisible() == true){
                    if(checkOnTarget() == false){
                        adjustHeading(Current_X, Current_Y, Goal_X, Goal_Y, Current_Theta);
                    } else {
                        MoveForward();
                    }
                } else {
                    MoveForward();
                }
            }
        }
        
    } else {
        if(!AngleLocked){
            ComputeReboundAngle();
            AngleLocked = true;
            Rebound = true;
        }
        AdjustAngle(target_angle);

        
    }

    if(CheckForDestination()){
        stopMoving();
    } else {
        plotPath();
    }
}

