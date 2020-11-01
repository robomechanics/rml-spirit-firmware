#include "clark_trot_controller/clark_trot_controller.h"

ClarkTrotController::ClarkTrotController(ros::NodeHandle nh) {
	nh_ = nh;

  // Setup pubs and subs

  joint1_pub_ = nh_.advertise<std_msgs::Float64>("/spirit/",1);
}

void ClarkTrotController::spin() {
  double start_time = ros::Time::now().toSec();

	ros::Rate r(update_rate_);
	while (ros::ok()) {
    double elapsed_time = ros::Time::now().toSec() - start_time;
    this->sendJointPositions(elapsed_time);
		ros::spinOnce();
		r.sleep();
	}
}

void ClarkTrotController::sendJointPositions(double &elapsed_time)
{

}