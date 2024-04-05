#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	void SetLinearVelocity(const float &vel);

	void SetAngularVelocity(const float &ang);

	void PControllerSpeed(const float &dist);

	void PControllerHeading(const float &ang);

	void DriveRobot();

	bool PositionInvalid();

	void VisualizeMarker();

	void InitMarker();


	/*!
	 * Callback Function
	*/
	void ScanCallback(const sensor_msgs::LaserScanConstPtr& msg);


	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber scan_subscriber_;
	ros::Publisher cmd_vel_pub_;
	ros::Publisher marker_pub_;
	geometry_msgs::Twist msg_;
	visualization_msgs::Marker marker_;
	float p_gain_vel_;
	float p_gain_ang_;
	double min_range_;
	double max_range_;
	double laser_scan_min_height_;
	double laser_scan_max_height_;
	float pillar_position_[2];

};

} /* namespace */
