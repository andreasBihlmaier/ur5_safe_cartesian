#include "UR5SafeCartesian.h"

// system includes

// library includes
#include <tf/transform_datatypes.h>

// custom includes
#include <safe_cartesian_moveit/CollisionCheckMoveIt.hpp>
#include <ur_kinematics/ur_kin.h>


double
joint_dist(const sensor_msgs::JointState& j1, const sensor_msgs::JointState& j2)
{
  if (j1.position.size() != j2.position.size()) {
    ROS_ERROR("joint_dist: Unequal number of joints");
    return INFINITY;
  }

  double dist = 0;
  for (size_t joint_idx = 0; joint_idx < j1.position.size(); joint_idx++) {
    double d = j1.position[joint_idx] - j2.position[joint_idx];
    dist += d * d;
  }
  dist = sqrt(dist);

  return dist;
}

/*---------------------------------- public: -----------------------------{{{-*/
const std::string UR5SafeCartesian::jointNames[UR5_JOINTS] = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" };
const double UR5SafeCartesian::jointLimits[UR5_JOINTS] = { 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI };

UR5SafeCartesian::UR5SafeCartesian(const std::string& p_robotName)
  :m_robotName(p_robotName),
   m_gpi(UR5_JOINTS),
   m_gpiPosCurrentBuffer(UR5_JOINTS, 0),
   m_gpiPosTargetBuffer(UR5_JOINTS, 0),
   m_gpiPosMinBuffer(UR5_JOINTS, 0),
   m_gpiPosMaxBuffer(UR5_JOINTS, 0),
   m_gpiVelCurrentBuffer(UR5_JOINTS, 0),
   m_gpiVelMaxBuffer(UR5_JOINTS, m_velMax),
   m_gpiAccelMaxBuffer(UR5_JOINTS, m_accelMax)
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
  m_targetJointState.name = m_jointNames;

  m_collision_check = new CollisionCheckMoveIt();
  // gpi
  for (size_t jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
    m_gpiPosMinBuffer[jointIdx] = -1 * M_PI;
    m_gpiPosMaxBuffer[jointIdx] = M_PI;
  }
  m_gpi.setXTarget(m_gpiPosTargetBuffer);
  m_gpi.setXLast(m_gpiPosCurrentBuffer);
  m_gpi.setVLast(m_gpiVelCurrentBuffer);
  m_gpi.setXMin(m_gpiPosMinBuffer);
  m_gpi.setXMax(m_gpiPosMaxBuffer);
  m_gpi.setVMax(m_gpiVelMaxBuffer);
  m_gpi.setAMax(m_gpiAccelMaxBuffer);
  m_gpi.setDt(0.05);
  m_gpi.setMode(1);

  m_setJointTopicSub = m_node.subscribe<sensor_msgs::JointState>("set_joint", 1, &UR5SafeCartesian::setJointCallback, this);
  m_getJointTopicPub = m_node.advertise<sensor_msgs::JointState>("get_joint", 1);
  m_setCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>("set_cartesian", 1, &UR5SafeCartesian::setCartesianCallback, this);
  m_setUnsafeCartesianTopicSub = m_node.subscribe<geometry_msgs::Pose>("unsafe/set_cartesian", 1, &UR5SafeCartesian::setUnsafeCartesianCallback, this);
  m_getCartesianTopicPub = m_node.advertise<geometry_msgs::Pose>("get_cartesian", 1);
  m_stateTopicPub = m_node.advertise<std_msgs::String>("state", 1);

  m_directGetJointTopicSub = m_node.subscribe<sensor_msgs::JointState>("direct/get_joint", 1, &UR5SafeCartesian::directGetJointCallback, this);
  m_directSetJointTopicPub = m_node.advertise<sensor_msgs::JointState>("direct/set_joint", 1);
  m_directStateTopicSub = m_node.subscribe<std_msgs::String>("direct/state", 1, &UR5SafeCartesian::directStateCallback, this);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
bool
UR5SafeCartesian::pathHasCollision(const sensor_msgs::JointState& targetJointState)
{
  if (joint_dist(m_lastJointState, targetJointState) < path_collision_check_dist_threshold) {
    return m_collision_check->hasCollision(targetJointState);
  } else {
    ROS_INFO("Checking PATH for collisions");
    for (size_t jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
      m_gpiPosTargetBuffer[jointIdx] = targetJointState.position[jointIdx];
    }
    m_gpi.setXTarget(m_gpiPosTargetBuffer);

    std::vector<sensor_msgs::JointState> targetJointStateVec;
    while (true) {
      m_gpi.interpolate();
      m_gpi.getXNow(m_gpiPosCurrentBuffer);

      // target not reachable
      if (targetJointStateVec.size() != 0 && std::equal(m_gpiPosCurrentBuffer.begin(), m_gpiPosCurrentBuffer.end(), targetJointStateVec[targetJointStateVec.size() - 1].position.begin())) {
        ROS_INFO("Target not reachable, counting this as collision");
        return true;
      }

      sensor_msgs::JointState intermediate_state(targetJointState);
      intermediate_state.position = m_gpiPosCurrentBuffer;
      targetJointStateVec.push_back(intermediate_state);

      // target reached
      if (std::equal(m_gpiPosCurrentBuffer.begin(), m_gpiPosCurrentBuffer.end(), targetJointState.position.begin())) {
        break;
      }
    }

    return m_collision_check->hasPathCollision(targetJointStateVec);
  }
}

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

  if (pathHasCollision(*jointsMsg)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    m_currentState.data = "SAFE_UR5_ERROR|SAFE_UR5_COLLISION";
    m_stateTopicPub.publish(m_currentState);
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
UR5SafeCartesian::doCartesian(const geometry_msgs::Pose::ConstPtr& poseMsg, bool collision_checking)
{
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
  //tprint(T);

  double joint_solutions[8*6];
  unsigned joint_solutions_count = ur_kinematics::inverse(T, joint_solutions, m_lastJointState.position[5]);
  //printf("raw solutions:\n");
  //jsolprint(joint_solutions, joint_solutions_count);
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
    //printf("joint_optimal_interval: ");
    for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
      if (isnan(joint_solutions[rowmajoridx(solutionIdx,jointIdx,UR5_JOINTS)])) {
        //printf("NaN!");
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
      //printf("%2.3lf ", ival);
    }
    //printf("\n");
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
    //printf("Solution %d dist=%lf\n", solutionIdx, dist);
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
  //printf("optimal interval solutions:\n");
  //jsolprint(joint_solutions, joint_solutions_count);
  //printf("Using solution %d\n", minDistSolutionIdx);

  sensor_msgs::JointState unsafeTargetJointState(m_targetJointState);
  for (unsigned jointIdx = 0; jointIdx < UR5_JOINTS; jointIdx++) {
    unsafeTargetJointState.position[jointIdx] = joint_solutions[rowmajoridx(minDistSolutionIdx,jointIdx,UR5_JOINTS)];
  }
  //std::cout << "unsafeTargetJointState: " << unsafeTargetJointState << std::endl;

  if (collision_checking && pathHasCollision(unsafeTargetJointState)) {
    std::cout << "------------------> COLLISION <---------------" << std::endl;
    m_currentState.data = "SAFE_UR5_ERROR|SAFE_UR5_COLLISION";
    m_stateTopicPub.publish(m_currentState);
    return;
  }

  m_targetCartesianPose = *poseMsg;
  m_targetJointState = unsafeTargetJointState;
  publishToHardware();
}


void
UR5SafeCartesian::setCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
  //std::cout << "setCartesianCallback: poseMsg=\n" << *poseMsg << std::endl;
  doCartesian(poseMsg);
}


void
UR5SafeCartesian::setUnsafeCartesianCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
{
  //std::cout << "setUnsafeCartesianCallback: poseMsg=\n" << *poseMsg << std::endl;
  doCartesian(poseMsg, false);
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
