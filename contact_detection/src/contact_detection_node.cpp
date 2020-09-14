#include <ros/ros.h>
#include "contact_detection/contact_detection.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "contact_detection_node");
	ros::NodeHandle nh;

	contact_detection cd(nh);

	return 0;
}
