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

class GeoController {

public:
    GeoController(const std::string& worldFrame, const std::string& frame,
                  const ros::NodeHandle& n): m_serviceTakeoff(), m_serviceLand(), m_state(Idle), m_thrust(0),
                                             m_startZ(0.0), m_worldFrame(worldFrame), m_bodyFrame(frame)

    {
        ros::NodeHandle nh;
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_serviceTakeoff = nh.advertiseService("takeoff", &GeoController::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &GeoController::land, this);

    };


    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &GeoController::iteration, this);
        ros::spin();
    }


    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
            case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
                {
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
                break;
            case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
                // intentional fall-thru
            case Automatic:
            {
                // get the current position of the drone
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_bodyFrame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                        tf::Quaternion(
                                targetDrone.pose.orientation.x,
                                targetDrone.pose.orientation.y,
                                targetDrone.pose.orientation.z,
                                targetDrone.pose.orientation.w
                        )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);

                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);


            }
                break;
            case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
                break;
        }
    }


private:
    State m_state;
    ros::Publisher m_pubNav;
    std::string m_worldFrame;
    std::string m_bodyFrame;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    tf::TransformListener m_listener;
    float m_thrust;
    float m_startZ; // only for landing to the start z value
    geometry_msgs::PoseStamped m_goal;

    bool takeoff(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("Takeoff requested!");
        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();
        return true;
    }

    bool land(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        ROS_INFO("Landing requested!");
        m_state = Landing;
        return true;
    }

};





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

    GeoController controller(worldFrame, frame, n);
    controller.run(frequency);


    return 0;
}
