#include <ros/ros.h>
#include <mavel/mavel.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavel");
	Mavel mavel;

	ros::Rate loopRate( mavel.get_rate() );

	while( ros::ok() ) {
		mavel.step();

		ros::spinOnce();	//Not needed here, but will update subscribers if they are added
		loopRate.sleep();
	}

	return 0;
}
