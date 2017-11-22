//
// Created by malintha on 11/22/17.
//
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

enum State
{
    Idle = 0,
    Automatic = 1,
    TakingOff = 2,
    Landing = 3,
};


bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
{
    ROS_INFO("Takeoff requested!");

    tf::StampedTransform transform;
//    m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
//    m_startZ = transform.getOrigin().z();

    return true;
}

bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
{
    ROS_INFO("Landing requested!");
//    m_state = Landing;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    static ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");
    std::string frame;
    n.getParam("frame", frame);
    double frequency;
    n.param("frequency", frequency, 50.0);

    ros::Rate rate = 30;
    int frame_count = 0;
    float m_thrust = 40000;
    ros::Publisher m_pubNav;
    ros::NodeHandle nh;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;

    m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    m_serviceTakeoff = nh.advertiseService("takeoff", &takeoff);
    m_serviceLand = nh.advertiseService("land", &land);

    while(ros::ok()){
        rate.sleep();
        ROS_INFO("Frame: %d bodyFrame: %s thrust: %f", frame_count++, frame.data(), m_thrust);
        geometry_msgs::Twist msg;
        msg.linear.z = m_thrust;
        m_pubNav.publish(msg);
        ros::spinOnce();
    }


    return 0;
}
