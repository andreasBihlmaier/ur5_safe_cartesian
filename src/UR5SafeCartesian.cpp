#include "UR5SafeCartesian.h"

// system includes

// library includes
#include <tf/transform_datatypes.h>

// custom includes
#include <safe_cartesian_moveit/CollisionCheckMoveIt.hpp>
#include <ur_kinematics/ur_kin.h>


/*---------------------------------- public: -----------------------------{{{-*/
const std::string UR5SafeCartesian::jointNames[UR5_JOINTS] = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
const double UR5SafeCartesian::jointLimits[UR5_JOINTS] = { 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI };

UR5SafeCartesian::UR5SafeCartesian(const std::string& p_robotName, const std::string& p_setJointTopic, const std::string& p_getJointTopic, const std::string& p_setCartesianTopic, const std::string& p_getCartesianTopic, const std::string& p_stateTopic, const std::string& p_directSetJointTopic, const std::string& p_directGetJointTopic, const std::string& p_directStateTopic)
  :m_robotName(p_robotName),
   m_setJointTopic(p_setJointTopic),
   m_getJointTopic(p_getJointTopic),
   m_setCartesianTopic(p_setCartesianTopic),
   m_getCartesianTopic(p_getCartesianTopic),
   m_stateTopic(p_stateTopic),
   m_directSetJointTopic(p_directSetJointTopic),
   m_directGetJointTopic(p_directGetJointTopic),
   m_directStateTopic(p_directStateTopic)
{
  // ros
  m_lastJointState.position.resize(UR5_JOINTS, 0);
  m_lastJointState.velocity.resize(UR5_JOINTS, 0);
  m_lastJointState.effort.resize(UR5_JOINTS, 0);
  m_targetJointState.position.resize(UR5_JOINTS, 0);
  m_targetJointState.velocity.resize(UR5_JOINTS, 0);
  m_targetJointState.effort.resize(UR5_JOINTS, 0);

  m_jointNames.resize(UR5_JOINTS);
  for (size_t jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
    m_jointNames[jointIdx] = /*m_robotName + "_" +*/ jointNames[jointIdx] /*+ "_joint"*/;
  }

  m_setJointTopicSub = m_node.subscribe<sensor_msgs::JointState>(m_setJointTopic, 1, &UR5SafeCartesian::setJointCallback, this);
  m_getJointTopicPub = m_node.advertise<sensor_msgs::JointState>(m_getJointTopic, 1);
  m_setCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>(m_setCartesianTopic, 1, &UR5SafeCartesian::setCartesianCallback, this);
  m_getCartesianTopicPub = m_node.advertise<geometry_msgs::Pose>(m_getCartesianTopic, 1);
  m_stateTopicPub = m_node.advertise<std_msgs::String>(m_stateTopic, 1);

  m_directGetJointTopicSub = m_node.subscribe<sensor_msgs::JointState>(m_directGetJointTopic, 1, &UR5SafeCartesian::directGetJointCallback, this);
  m_directSetJointTopicPub = m_node.advertise<sensor_msgs::JointState>(m_directSetJointTopic, 1);
  m_directStateTopicSub = m_node.subscribe<std_msgs::String>(m_directStateTopic, 1, &UR5SafeCartesian::directStateCallback, this);

  m_collision_check = new CollisionCheckMoveIt();
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
UR5SafeCartesian::setJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
{
  //std::cout << "setJointCallback: jointsMsg=\n" << *jointsMsg << std::endl;

  if (jointsMsg->position.size() != UR5_JOINTS) {
    m_currentState.data = "SAFE_UR5_ERROR|SAFE_UR5_WRONG_NUMBER_OF_JOINTS";
    m_stateTopicPub.publish(m_currentState);
    return;
  }

  for (size_t jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
    if (std::abs(jointsMsg->position[jointIdx]) > jointLimits[jointIdx]) {
      m_currentState.data = "SAFE_UR5_ERROR|SAFE_UR5_JOINT_LIMIT_EXCEEDED";
      m_stateTopicPub.publish(m_currentState);
      return;
    }
  }

  if (m_collision_check->hasCollision(*jointsMsg)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    return;
  }

  m_targetJointState = *jointsMsg;
  publishToHardware();
}

int
rowmajoridx(int row, int col, int numcols)
{
  return row * numcols + col;
}

void
tprint(const double* T)
{
  printf("T:\n");
  for (unsigned row = 0; row < 4; row++) {
    for (unsigned col = 0; col < 4; col++) {
      printf("%2.3lf ", T[rowmajoridx(row,col,4)]);
    }
    printf("\n");
  }
  printf("\n");
}

void
jsolprint(const double* joint_solutions, unsigned joint_solutions_count)
{
  printf("joint_solutions:\n");
  for (unsigned solutionIdx = 0; solutionIdx < joint_solutions_count; solutionIdx++) {
    printf("%d: ", solutionIdx);
    for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
      printf("%2.3lf ", joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)]);
    }
    printf("\n");
  }
  printf("\n");
}

void
UR5SafeCartesian::setCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
  //std::cout << "setCartesianCallback: poseMsg=\n" << *poseMsg << std::endl;

  tf::Pose tfpose;
  tf::poseMsgToTF(*poseMsg, tfpose);

  double T[4*4];
  for (unsigned col = 0; col < 3; col++) {
    T[rowmajoridx(3,col,4)] = 0;
  }
  T[rowmajoridx(3,3,4)] = 1;
  tf::Matrix3x3 rotationMatrix = tfpose.getBasis();
  for (unsigned row = 0; row < 3; row++) {
    for (unsigned col = 0; col < 3; col++) {
       T[rowmajoridx(row,col,4)] = rotationMatrix[row][col];
    }
  }
  tf::Vector3 translationVector = tfpose.getOrigin();
  for (unsigned row = 0; row < 3; row++) {
    T[rowmajoridx(row,3,4)] = translationVector[row];
  }
  tprint(T);

  double joint_solutions[8*6];
  unsigned joint_solutions_count = ur_kinematics::inverse(T, joint_solutions, m_lastJointState.position[5]);
  printf("raw solutions:\n");
  jsolprint(joint_solutions, joint_solutions_count);
  if (joint_solutions_count == 0) {
    m_currentState.data = "SAFE_UR5_ERROR|SAFE_UR5_NO_KINEMATICS_SOLUTION";
    m_stateTopicPub.publish(m_currentState);
    return;
  }

  // use solution closest to current pos (in joint space)
  int minDistSolutionIdx = -1;
  double minDistSolution = 10e6;
  for (unsigned solutionIdx = 0; solutionIdx < joint_solutions_count; solutionIdx++) {
    bool validSolution = true;
    // ur_kinematics::inverse returns angles in [0,2*PI) since all axis are +-2*PI, we want to use value of interval we are in
    printf("joint_optimal_interval: ");
    for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
      if (isnan(joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)])) {
        printf("NaN!");
        validSolution = false;
        break;
      }
      double distpos = abs(m_lastJointState.position[jointIdx] - joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)]); 
      double distneg = abs(m_lastJointState.position[jointIdx] - (joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)] - 2*M_PI)); 
      double ival;
      if (distneg < distpos) {
        ival = -2 * M_PI;
      } else {
        ival = 0;
      }
      joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)] += ival;
      printf("%2.3lf ", ival);
    }
    printf("\n");
    if (!validSolution) {
      continue;
    }

    double dist = 0;
    for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
      double d = joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)] - m_lastJointState.position[jointIdx];
      while (d > M_PI) {
        d -= 2*M_PI;
      }
      while (d < -M_PI) {
        d += 2*M_PI;
      }
      dist += d * d;
    }
    printf("Solution %d dist=%lf\n", solutionIdx, dist);
    if (dist < minDistSolution) {
      minDistSolution = dist;
      minDistSolutionIdx = solutionIdx;
    }
  }
  if (minDistSolutionIdx == -1) {
    m_currentState.data = "SAFE_UR5_ERROR|SAFE_UR5_NO_KINEMATICS_SOLUTION";
    m_stateTopicPub.publish(m_currentState);
    return;
  }
  printf("optimal interval solutions:\n");
  jsolprint(joint_solutions, joint_solutions_count);
  printf("Using solution %d\n", minDistSolutionIdx);

  sensor_msgs::JointState unsafeTargetJointState(m_targetJointState);
  for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
    unsafeTargetJointState.position[jointIdx] = joint_solutions[rowmajoridx(minDistSolutionIdx,jointIdx,UR5_JOINTS)];
  }
  if (m_collision_check->hasCollision(unsafeTargetJointState)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    return;
  }

  m_targetCartesianPose = *poseMsg;
  m_targetJointState = unsafeTargetJointState;
  publishToHardware();
}

void
UR5SafeCartesian::directGetJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
{
  //std::cout << "directGetJointCallback: jointsMsg=\n" << *jointsMsg << std::endl;
 
  m_lastJointState = *jointsMsg;
  m_lastJointState.name = m_jointNames;

  double T[4*4];
  ur_kinematics::forward(&m_lastJointState.position[0], T);
  //tprint(T);
  tf::Transform cartesianPose;
  tf::Matrix3x3 rotationMatrix;
  for (unsigned row = 0; row < 3; row++) {
    for (unsigned col = 0; col < 3; col++) {
       rotationMatrix[row][col] = T[rowmajoridx(row,col,4)];
    }
  }
  tf::Vector3 translationVector;
  for (unsigned row = 0; row < 3; row++) {
    translationVector[row] = T[rowmajoridx(row,3,4)];
  }
  cartesianPose.setBasis(rotationMatrix);
  cartesianPose.setOrigin(translationVector);
  tf::poseTFToMsg(cartesianPose, m_lastCartesianPose);
 
  publishToApplication();
  publishToTF();
}

void
UR5SafeCartesian::directStateCallback(const std_msgs::String::ConstPtr& stateMsg)
{
  std::cout << "directStateCallback: state=\n" << *stateMsg << std::endl;
}

void
UR5SafeCartesian::publishToHardware()
{
  m_targetJointState.header.stamp = ros::Time::now();
  m_directSetJointTopicPub.publish(m_targetJointState);
}

void
UR5SafeCartesian::publishToApplication()
{
  m_lastJointState.header.stamp = ros::Time::now();
  m_getJointTopicPub.publish(m_lastJointState);
  m_getCartesianTopicPub.publish(m_lastCartesianPose);
}

void
UR5SafeCartesian::publishToTF()
{
}
/*------------------------------------------------------------------------}}}-*/
