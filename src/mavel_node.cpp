#include <ros/ros.h>
#include <mavel/mavel.h>
#include <signal.h>


void ShutdownSigintHandler( int sig ) {
	// Do some custom action.
	// For example, publish a stop message to some other nodes.
	mavel.shutdown();

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavel");
	Mavel mavel;

	signal(SIGINT, ShutdownSigintHandler);

	ros::spin();

	/*

	std::printf("test2");
	*/
	return 0;
}
