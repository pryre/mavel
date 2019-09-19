#include <ros/ros.h>
#include <mavel/mavel.h>
#include <signal.h>

static Mavel* _mavel;

static void cleanup( void ) {
	if (_mavel != NULL) {
		_mavel->shutdown();
		_mavel = NULL;
	}
}

static void shutdown_sigint_handler( int sig ) {
	//Clean up allocated variables
	cleanup();

	// Clean up the rest of ROS
	ros::shutdown();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavel");
	Mavel mavel;
	_mavel = &mavel;

	signal(SIGINT, shutdown_sigint_handler);
	ros::spin();

	//Clean up allocated variables
	cleanup();

	return 0;
}
