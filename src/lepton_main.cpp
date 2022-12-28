#include <iostream>
#include <ros/ros.h>
#include "lepton_node.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flir_thermal_driver_lepton" );
    ros::NodeHandle private_node("~");
    
    LeptonNode lepton(private_node);
    if ( ! lepton.isRunning() )
    {
        ROS_ERROR_STREAM("Thermal data capturing from Lepton did not start.");
        return -1;
    }
    ros::spin();
    return 0;
}

