#include <iostream>
#include "AutoDrive.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "AutoDrive"); //!< Initialise the ROS node
    drive control; //!< Create the drive object
    
    
    ros::Rate loop_rate(30); //!< Set the speed of the system

    while(ros::ok()) //!< Begin the loop of the system
    {
        //control.Debug();
        control.Control();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}