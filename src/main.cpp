#include "UR5SafeCartesian.h"
#include <getopt.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_safe_cartesian");

  std::string robotName = "ur5";
  std::string setJointTopic = "set_joint";
  std::string getJointTopic = "get_joint";
  std::string setCartesianTopic = "set_cartesian";
  std::string getCartesianTopic = "get_cartesian";
  std::string stateTopic = "state";

  std::string directSetJointTopic = "direct/set_joint";
  std::string directGetJointTopic = "direct/get_joint";
  std::string directStateTopic = "direct/state";

  const char optstring[] = "";
  struct option longopts[] = {
    { "robotname", required_argument, NULL, 0 },
  };
  int opt;
  int optindex;
  while ((opt = getopt_long(argc, argv, optstring, longopts, &optindex)) != -1) {
    switch (opt) {
    case 0:
      if (strcmp(longopts[optindex].name, "robotname") == 0) {
        robotName = optarg;
      }
      break;
    }
  }

  UR5SafeCartesian ur5SafeCartesian(robotName);

  std::cout << "Spinning" << std::endl;
  ros::spin();

  return 0;
}
