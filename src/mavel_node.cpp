#include <ros/ros.h>
#include <mavel/mavel.h>
#include <signal.h>

static Mavel* _mavel;

static void cleanup( void ) {
	if (_mavel != NULL)
		delete _mavel;
}

static void shutdown_sigint_handler( int sig ) {
	//Clean up allocated variables
	cleanup();

	// Clean up the rest of ROS
	ros::shutdown();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavel");
	_mavel = new Mavel();

	signal(SIGINT, shutdown_sigint_handler);

	ros::spin();

	//Clean up allocated variables
	cleanup();

	return 0;
}
