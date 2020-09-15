#include "clark_trot_controller/clark_trot_controller.h"

ClarkTrotController::ClarkTrotController(ros::NodeHandle nh) {
	nh_ = nh;

  // Load rosparams from parameter server
  std::string joint_encoder_topic, control_torques_topic;
  nh.param<std::string>("topics/joint_encoder", "/joint_encoder");
  nh.param<std::string>("topics/control_torques", "/control_torques");

  // Setup pubs and subs
  joint_encoder_sub_ = nh_.subscribe(joint_encoder_topic,1,&ClarkTrotController::jointEncoderCallback, this);
  control_torques_pub_ = nh_.advertise<spirit_msgs::TorqueCommand>(control_torques_topic,1);
}

void ClarkTrotController::jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  
}

void ClarkTrotController::spin() {
	ros::Rate r(update_rate_);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}

