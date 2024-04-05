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
      || !nodeHandle_.getParam("P_gain_vel", p_gain_vel_)
      || !nodeHandle_.getParam("P_gain_ang", p_gain_ang_)){
      ROS_ERROR("Could not find subscriber parameters!");
  }

  //TODO: EX2.4,5
  scan_subscriber_ = nodeHandle_.subscribe(topic, queue_size, &SmbHighlevelController::ScanCallback, this);
  //TODO: EX3.4
  cmd_vel_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  InitMarker();
}

void SmbHighlevelController::SetLinearVelocity(const float &vel) {
  msg_.linear.x = vel;
}

void SmbHighlevelController::SetAngularVelocity(const float &ang) {
  msg_.angular.z = ang;
}

void SmbHighlevelController::PControllerSpeed(const float &dist) {
  float vel = p_gain_vel_ * (dist - 0.1);
  if (!PositionInvalid()) {
    if (vel > 6.0) {
      vel = 6.0;
    }
    else if (vel < 0.0) {
      vel = 0.0;
      SetAngularVelocity(0.0);
    }
  }
  else {
    vel = 0.0;
  }
  SetLinearVelocity(vel);
}

void SmbHighlevelController::PControllerHeading(const float &ang) {
  if (!PositionInvalid()) {
    float diff = -ang;
    SetAngularVelocity(p_gain_ang_ * diff);
  }
}

void SmbHighlevelController::DriveRobot() {
  cmd_vel_pub_.publish(msg_);
}

bool SmbHighlevelController::PositionInvalid() {
  return (pillar_position_[0] == std::numeric_limits<float>::infinity() * -1 ||
          pillar_position_[1] == std::numeric_limits<float>::infinity() * -1);
}

//TODO: EX3.8
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
  marker_.scale.x = 1.0;
  marker_.scale.y = 1.0;
  marker_.scale.z = 1.0;
  marker_.color.a = 1.0;
  marker_.color.r = 255.0;
  marker_.color.g = 0.0;
  marker_.color.b = 0.0;
}


void SmbHighlevelController::ScanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
  //TODO: Ex2.6
	auto minimum_distance_ptr = std::min_element(msg->ranges.cbegin(), msg->ranges.cend());
  int count = minimum_distance_ptr - msg->ranges.cbegin();
  auto angular = msg->angle_min + msg->angle_increment * count;
  //TODO: Ex3.3
  pillar_position_[0] = *minimum_distance_ptr * std::cos(angular);
  pillar_position_[1] = *minimum_distance_ptr * std::sin(angular);

  ROS_INFO_STREAM("Pillar is " << *minimum_distance_ptr << "away at" << angular << " degrees");
  ROS_INFO_STREAM("Pillar's position wrt robot is: " << pillar_position_[0] << ","
                                                     << pillar_position_[1]);
  
  PControllerHeading(angular);
  PControllerSpeed(*minimum_distance_ptr);

  DriveRobot();
  
  VisualizeMarker();
}


SmbHighlevelController::~SmbHighlevelController()
{
}

} /* namespace */
