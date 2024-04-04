#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>


namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  std::string topic;
  int queue_size;
  if(!nodeHandle_.getParam("subscriber_topic", topic)
      || !nodeHandle_.getParam("queue_size", queue_size)
      || !nodeHandle_.getParam("P_gain", p_gain_)){
      ROS_ERROR("Could not find subscriber parameters!");
  }

  scan_subscriber_ = nodeHandle_.subscribe(topic, queue_size, &SmbHighlevelController::ScanCallback, this);
  cmd_vel_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  InitMarker();
}

void SmbHighlevelController::SetVelocity(const float &vel_x, const float &vel_y) {
  msg_.linear.x = vel_x;
  msg_.linear.y = vel_y;
}

void SmbHighlevelController::PController(){
  float vel_x = p_gain_ * (pillar_position_[0] - 0.01);
  float vel_y = p_gain_ * (pillar_position_[1] - 0.01);
  if(vel_x > 5.0) {
    vel_x = 5.0;
  }
  else if(vel_x < 0.0) {
    vel_x = 0.0;
  }
  if(vel_y > 5.0) {
    vel_y = 5.0;
  }
  else if(vel_y < 0.0) {
    vel_y = 0.0;
  }
  SetVelocity(vel_x, vel_y);
  cmd_vel_pub_.publish(msg_);
}

void SmbHighlevelController::VisualizeMarker() {
  marker_.pose.position.x = pillar_position_[0];
  marker_.pose.position.y = pillar_position_[1];
  marker_.pose.position.z = -1;
  marker_pub_.publish(marker_);
}

void SmbHighlevelController::InitMarker(){
  marker_.header.frame_id = "base_laser";
  marker_.header.stamp = ros::Time();
  marker_.ns = "pillar_marker";
  marker_.id = 1;
  marker_.type = visualization_msgs::Marker::CYLINDER;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = pillar_position_[0];
  marker_.pose.position.y = pillar_position_[1];
  marker_.pose.position.z = -1.0;
  marker_.scale.x = 0.2;
  marker_.scale.y = 0.2;
  marker_.scale.z = 0.2;
  marker_.color.a = 1.0;
  marker_.color.r = 1.0;
  marker_.color.g = 0.0;
  marker_.color.b = 0.0;
}


void SmbHighlevelController::ScanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
  //find the minimum distance
	auto minimum_distance_ptr = std::min_element(msg->ranges.cbegin(), msg->ranges.cend());
  int count = minimum_distance_ptr - msg->ranges.cbegin();
  auto angular = msg->angle_min + msg->angle_increment * count;
  pillar_position_[0] = *minimum_distance_ptr * std::cos(angular);
  pillar_position_[1] = *minimum_distance_ptr * std::sin(angular);
  PController();
  VisualizeMarker();
}


SmbHighlevelController::~SmbHighlevelController()
{
}

} /* namespace */
