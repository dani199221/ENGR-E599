//
// Created by malintha on 11/22/17.
//
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    // Read parameters
    static ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");
    std::string frame;
    n.getParam("frame", frame);
    double frequency;
    n.param("frequency", frequency, 50.0);

//    Controller controller(worldFrame, frame, n);
//    controller.run(frequency);
    ros::Rate rate = 30;
    int frame_count = 0;

    while(ros::ok()){
        rate.sleep();
        ROS_INFO("Frame: %d", frame_count++);
        ros::spinOnce();
    }


    return 0;
}