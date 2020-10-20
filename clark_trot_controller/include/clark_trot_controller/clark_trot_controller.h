#ifndef CLARK_TROT_CONTROLLER_H
#define CLARK_TROT_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
//! Implements open loop clark trot controller
/*!
   ClarkTrotController implements all control logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class ClarkTrotController {
  public:
	/**
	 * @brief Constructor for ClarkTrotController
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type ClarkTrotController
	 */
	ClarkTrotController(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:

	/// Publisher for desired joint positions
	ros::Publisher joint0_pub_,joint1_pub_,joint2_pub_,joint3_pub_,joint4_pub_,
					joint5_pub_,joint6_pub_,joint7_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;
};


#endif // CLARK_TROT_CONTROLLER_H
