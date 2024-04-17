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

    //set the linear_x velocity
	void SetLinearVelocity(const float &vel);

	//set the angular_z velocity
	void SetAngularVelocity(const float &ang);

	//p controller for the linear_x velocity
	void PControllerSpeed(const float &dist);

	//p controller for the angular_z velocity
	void PControllerHeading(const float &ang);

	//publish the velocity
	void DriveRobot();

	//check whether the pillar's position is valid
	bool PositionInvalid();

	//visualize the pillar
	void VisualizeMarker();

	//initialize the marker
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
	float pillar_position_[2];

};

} /* namespace */
