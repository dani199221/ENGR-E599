//
// Created by malintha on 11/22/17.
//
#include "ros/ros.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    // Read parameters
    ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");
    std::string frame;
    n.getParam("frame", frame);
    double frequency;
    n.param("frequency", frequency, 50.0);

//    Controller controller(worldFrame, frame, n);
//    controller.run(frequency);
    ros::
    while(ros::ok()){

    }


    return 0;
}