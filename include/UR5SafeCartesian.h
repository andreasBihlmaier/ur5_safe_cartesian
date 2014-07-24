#ifndef _LWR_SAFE_CARTESIAN_H_
#define _LWR_SAFE_CARTESIAN_H_

// system includes

// library includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// custom includes
#include <GeneralPurposeInterpolator.hpp>

// forward declarations
class CollisionCheckMoveIt;

#define UR5_JOINTS 6

class UR5SafeCartesian
{
  public:
    // enums

    // typedefs

    // const static member variables
    static const std::string jointNames[UR5_JOINTS];
    static const double jointLimits[UR5_JOINTS];
    static const double path_collision_check_dist_threshold = 0.1;
    static const double m_velMax = 0.8;
    static const double m_accelMax = 10.0;
 
    // static utility functions


    // constructors
    UR5SafeCartesian(const std::string& p_robotName);

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
    bool pathHasCollision(const sensor_msgs::JointState& targetJointState);

    // variables
    std::string m_robotName;

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

    CollisionCheckMoveIt* m_collision_check;
    GeneralPurposeInterpolator m_gpi;
    std::vector<double> m_gpiPosCurrentBuffer;
    std::vector<double> m_gpiPosTargetBuffer;
    std::vector<double> m_gpiPosMinBuffer;
    std::vector<double> m_gpiPosMaxBuffer;
    std::vector<double> m_gpiVelCurrentBuffer;
    std::vector<double> m_gpiVelMaxBuffer;
    std::vector<double> m_gpiAccelMaxBuffer;
};

#endif // _LWR_SAFE_CARTESIAN_H_
