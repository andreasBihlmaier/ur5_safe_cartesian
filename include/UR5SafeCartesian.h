#ifndef _LWR_SAFE_CARTESIAN_H_
#define _LWR_SAFE_CARTESIAN_H_

// system includes

// library includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// custom includes

// forward declarations

#define UR5_JOINTS 6

class UR5SafeCartesian
{
  public:
    // enums

    // typedefs

    // const static member variables
    static const std::string jointNames[UR5_JOINTS];
    static const double jointLimits[UR5_JOINTS];
 
    // static utility functions


    // constructors
    UR5SafeCartesian(const std::string& p_robotName, const std::string& p_setJointTopic, const std::string& p_getJointTopic, const std::string& p_setCartesianTopic, const std::string& p_getCartesianTopic, const std::string& p_stateTopic, const std::string& p_directSetJointTopic, const std::string& p_directGetJointTopic, const std::string& p_directStateTopic);

    // overwritten methods

    // methods

    // variables


  private:
    // methods
    void setJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg);
    void setCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg);
    void directGetJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg);
    void directStateCallback(const std_msgs::String::ConstPtr& stateMsg);
    void publishToHardware();
    void publishToApplication();
    void publishToTF();

    // variables
    std::string m_robotName;
    std::string m_setJointTopic;
    std::string m_getJointTopic;
    std::string m_setCartesianTopic;
    std::string m_getCartesianTopic;
    std::string m_stateTopic;

    std::string m_directSetJointTopic;
    std::string m_directGetJointTopic;
    std::string m_directStateTopic;

    std::vector<std::string> m_jointNames;

    ros::NodeHandle m_node;
    ros::Subscriber m_setJointTopicSub;
    ros::Publisher m_getJointTopicPub;
    ros::Subscriber m_setCartesianTopicSub;
    ros::Publisher m_getCartesianTopicPub;
    ros::Publisher m_stateTopicPub;
    ros::Subscriber m_directGetJointTopicSub;
    ros::Publisher m_directSetJointTopicPub;
    ros::Subscriber m_directStateTopicSub;
    sensor_msgs::JointState m_lastJointState;
    sensor_msgs::JointState m_targetJointState;
    geometry_msgs::Pose m_lastCartesianPose;
    geometry_msgs::Pose m_targetCartesianPose;
    std_msgs::String m_currentState;
};

#endif // _LWR_SAFE_CARTESIAN_H_
