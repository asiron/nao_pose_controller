#include <nao_pose_controller/nao_pose_controller.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nao_pose_controller");
  SNT::ARG::NaoControllers::PathFollower pf;
  pf.run();
  return 0;
}   