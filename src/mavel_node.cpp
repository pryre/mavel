#include <ros/ros.h>
#include <mavel/mavel.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavel");
	Mavel mavel;

	ros::spin();

	return 0;
}
