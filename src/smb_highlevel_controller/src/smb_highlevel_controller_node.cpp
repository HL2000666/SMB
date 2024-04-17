#include <ros/ros.h>
#include "smb_highlevel_controller/SmbHighlevelController.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "smb_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  smb_highlevel_controller::SmbHighlevelController smbHighlevelController(nodeHandle);
  ROS_INFO_STREAM("Check......");
  //ros::Subscriber scan_subscriber_ = nodeHandle.subscribe("/scan", 10, &smb_highlevel_controller::SmbHighlevelController::ScanCallback, &smbHighlevelController);

  ros::spin();
  return 0;
}
