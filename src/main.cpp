#include "UR5SafeCartesian.h"
#include <getopt.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_safe_cartesian");

  std::string robotName = "ur5";
  std::string setJointTopic = "ur5/set_joint";
  std::string getJointTopic = "ur5/get_joint";
  std::string setCartesianTopic = "ur5/set_cartesian";
  std::string getCartesianTopic = "ur5/get_cartesian";
  std::string stateTopic = "ur5/state";

  std::string directSetJointTopic = "ur5/direct/set_joint";
  std::string directGetJointTopic = "ur5/direct/get_joint";
  std::string directStateTopic = "ur5/direct/state";

  const char optstring[] = "";
  struct option longopts[] = {
    { "robotname", required_argument, NULL, 0 },
    { "setjointtopic", required_argument, NULL, 0 },
    { "getjointtopic", required_argument, NULL, 0 },
    { "setcartesiantopic", required_argument, NULL, 0 },
    { "getcartesiantopic", required_argument, NULL, 0 },
    { "statetopic", required_argument, NULL, 0 },
    { "directsetjointtopic", required_argument, NULL, 0 },
    { "directgetjointtopic", required_argument, NULL, 0 },
    { "directstatetopic", required_argument, NULL, 0 },
  };
  int opt;
  int optindex;
  while ((opt = getopt_long(argc, argv, optstring, longopts, &optindex)) != -1) {
    switch (opt) {
    case 0:
      if (strcmp(longopts[optindex].name, "robotname") == 0) {
        robotName = optarg;
      } else if (strcmp(longopts[optindex].name, "setjointtopic") == 0) {
        setJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "getjointtopic") == 0) {
        getJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "setcartesiantopic") == 0) {
        setCartesianTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "getcartesiantopic") == 0) {
        getCartesianTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "statetopic") == 0) {
        stateTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "directsetjointtopic") == 0) {
        directSetJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "directgetjointtopic") == 0) {
        getJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "directstatetopic") == 0) {
        directStateTopic = optarg;
      }
      break;
    }
  }

  UR5SafeCartesian ur5SafeCartesian(robotName, setJointTopic, getJointTopic, setCartesianTopic, getCartesianTopic, stateTopic, directSetJointTopic, directGetJointTopic, directStateTopic);

  std::cout << "Spinning" << std::endl;
  ros::spin();

  return 0;
}
