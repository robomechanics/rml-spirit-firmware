#include "clark_trot_controller/clark_trot_controller.h"

ClarkTrotController::ClarkTrotController(ros::NodeHandle nh) {
	nh_ = nh;

  // Setup pubs and subs

  joint1_pub_ = nh_.advertise<std_msgs::Float64>("/spirit/",1);
}

void ClarkTrotController::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}

