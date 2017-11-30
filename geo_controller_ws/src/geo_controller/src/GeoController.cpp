//
// Created by malintha on 11/22/17.
//
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "dynamicsImpl.h"

enum State {
    Idle = 0,
    Automatic = 1,
    TakingOff = 2,
    Landing = 3,
};

class GeoController {

public:
    GeoController(const std::string &worldFrame, const std::string &frame,
                  const ros::NodeHandle &n) : m_serviceTakeoff(), m_serviceLand(), m_state(Idle), m_thrust(0),
                                              m_startZ(0.0), m_worldFrame(worldFrame), m_bodyFrame(frame) {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_bodyFrame, ros::Time(0), ros::Duration(10.0));
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_serviceTakeoff = nh.advertiseService("takeoff", &GeoController::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &GeoController::land, this);
        dynamics = new dynamicsImpl(n, m_worldFrame, m_bodyFrame);
    };


    void run(double frequency) {
        this->frequency = frequency;
        ros::Timer timer = node.createTimer(ros::Duration(1.0 / frequency), &GeoController::iteration, this);
        ros::spin();
    }


    void iteration(const ros::TimerEvent &e) {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch (m_state) {
            case TakingOff: {
                tf::StampedTransform transform;
                ROS_INFO("In taking off\n");
                geometry_msgs::Twist msg;
                msg.linear.x = 20000; //forward (pitch)
                msg.linear.z = 20000; //sideways (roll)
                m_pubNav.publish(msg);

//                m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
//                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
//                {
//                    m_state = Automatic;
//                    m_thrust = 0;
//                }
//                else
//                {
//                    m_thrust += 10000 * dt;
//                    geometry_msgs::Twist msg;
//                    msg.linear.z = m_thrust;
//                    m_pubNav.publish(msg);
//                }

            }
                break;
            case Landing: {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }

            case Automatic: {
                // get the current position of the drone
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);

                // get x, v, r, Omega from the dynamicsImpl - done
                dynamics->setdt(dt);
                Vector3d* x_arr = dynamics->get_x_v_Omega();
                Vector3d x = x_arr[0];
                Vector3d x_dot = x_arr[1];
                Vector3d Omega = x_arr[3];
                Matrix3d R = dynamics->getR();

                // controllerImpl.set(x, v, R, Omega)
                // controllerImpl.getForceVec(t)
                // controllerImpl.getMomentVec(t)


//                geometry_msgs::Twist msg;
//                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
//                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
//                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);



            }
                break;
            case Idle: {
//                ROS_INFO("In idle: Omega:%f \n", dynamics->temp_acc);
                dynamics->setdt(dt);

                Vector3d* x_arr = dynamics->get_x_v_Omega();
                Vector3d x = x_arr[0];
                Vector3d x_dot = x_arr[1];
                Vector3d Omega = x_arr[3];
                Matrix3d R = dynamics->getR();
//                ROS_INFO("###tf dx dy dz: %f,%f,%f, dt: %f", Omega[0], Omega[1], Omega[2], dt);
                std::cout<<"R: "<<R<<std::endl;
//                ROS_INFO("\n");

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
    dynamicsImpl *dynamics;
    float frequency;
    ros::NodeHandle node;

    bool takeoff(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        ROS_INFO("Takeoff requested!");
        tf::StampedTransform transform;
        m_state = TakingOff;
//        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
//        m_startZ = transform.getOrigin().z();
        return true;
    }

    bool land(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        ROS_INFO("Landing requested!");
        m_state = Landing;
        return true;
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    static ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");
    std::string frame;
    n.getParam("/crazyflie/geocontroller/frame", frame);
    double frequency;
    n.param("frequency", frequency, 20.0);

    ROS_INFO("### Initializing geocontroller. worldFrame:%s bodyFrame%s\n",worldFrame.data(), frame.data());

    GeoController controller(worldFrame, frame, n);
    controller.run(frequency);


    return 0;
}
