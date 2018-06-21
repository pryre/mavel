#pragma once

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <eigen3/Eigen/Dense>
#include <contrail/path_extract.h>
#include <pid_controller_lib/pidController.h>

#include <string>

//PID-based position, velocity and accleration controller for a UAV
//Output calculated with normalized thrust vector and orientation.
//	Note: If either external velocity or acceleration input is used,
//	then a z body rate is sent instead of a orientation (angular
//	acceleration around the z axis are integrated).
//
//		Position Control (Optional: set timout to 0.0):
//			Reference:
//				- Pose message
//				- TF frame
//			Goal:
//				- Pose message
//				- TF frame
//			Feedback:
//				- Pose message
//				- TF frame
//		Velocity Control (Optional: set timout to 0.0):
//			Reference:
//				- Twist message
//			Goal:
//				- Twist message
//			Feedback:
//				- Twist message
//		Acceleration control options:
//			Goal:
//				- Accel message
//			Feedback:
//				- Accel message

enum mavel_data_stream_states {
	HEALTH_OK = 0,
	HEALTH_TIMEOUT,
	HEALTH_UNKNOWN
};

template<typename streamDataT> struct mavel_data_stream {
	mavel_data_stream_states state;				//Whether the stream is reliable

	ros::Duration timeout;	//How long to wait before a timeout
	uint64_t count;			//The current data count since last timeout
	uint64_t stream_count;	//The data count needed to establish a stream

	std::string stream_topic;	//Name of the data stream (for console output)
	streamDataT data;		//Latest data from the stream
};

struct mavel_params_pid {
	double kp;
	double ki;
	double kd;

	double tau;

	double min;
	double max;
};

class Mavel {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		ros::Publisher pub_output_position_;
		ros::Publisher pub_output_velocity_;
		ros::Publisher pub_output_acceleration_;
		ros::Publisher pub_output_attitude_;

		ros::Subscriber sub_reference_state_;
		ros::Subscriber sub_reference_odometry_;
		ros::Subscriber sub_setpoint_position_;
		ros::Subscriber sub_setpoint_velocity_;
		ros::Subscriber sub_setpoint_acceleration_;

		ros::Timer timer_controller_;
		bool control_started_;
		bool control_fatal_;

		double param_rate_control_;
		double param_tilt_max_;
		double param_throttle_min_;
		double param_throttle_mid_;
		double param_throttle_max_;
		double param_land_vel_;
		double param_home_x_;
		double param_home_y_;
		double param_home_z_;

		//Rate in Hz
		//Required stream count is derived as 2*rate
		double param_stream_min_rate_reference_odometry_;
		double param_stream_min_rate_reference_state_;
		double param_stream_min_rate_setpoint_position_;
		double param_stream_min_rate_setpoint_velocity_;
		double param_stream_min_rate_setpoint_acceleration_;

		mavel_data_stream<nav_msgs::Odometry> stream_reference_odometry_;
		mavel_data_stream<mavros_msgs::State> stream_reference_state_;
		mavel_data_stream<geometry_msgs::PoseStamped> stream_setpoint_position_;
		mavel_data_stream<geometry_msgs::TwistStamped> stream_setpoint_velocity_;
		mavel_data_stream<geometry_msgs::AccelStamped> stream_setpoint_acceleration_;

		PathExtract ref_path_;
		pidController controller_pos_x_;
		pidController controller_pos_y_;
		pidController controller_pos_z_;
		pidController controller_vel_x_;
		pidController controller_vel_y_;
		pidController controller_vel_z_;

		double integrator_body_rate_z_;

		std::string param_control_frame_id_;

	public:
		Mavel( void );

		~Mavel( void );

		void reference_odometry_cb( const nav_msgs::Odometry msg_in );
		void reference_state_cb( const mavros_msgs::State msg_in );

		void setpoint_position_cb( const geometry_msgs::PoseStamped msg_in );
		void setpoint_velocity_cb( const geometry_msgs::TwistStamped msg_in );
		void setpoint_acceleration_cb( const geometry_msgs::AccelStamped msg_in );

		//Handles the controller loop
		void controller_cb( const ros::TimerEvent& timerCallback );
		void do_control( const ros::TimerEvent& timerCallback, mavros_msgs::AttitudeTarget &goal_att );
		void do_failsafe( const ros::TimerEvent& timerCallback, mavros_msgs::AttitudeTarget &goal_att );

	private:
		//Initializes the pid parameters for a controller
		//param_base_name should be set first (e.g. "~/pid/position/")
		void param_pid_init( mavel_params_pid* pid, std::string controller_name );

		//Initializes the stream parameters
		template<typename streamDataT>
		mavel_data_stream<streamDataT> stream_init( const double min_rate, const std::string topic );

		//Updates the stream information that new data has been recieved
		//New data should have already been added into the stream
		template<typename streamDataT>
		void stream_update( mavel_data_stream<streamDataT> &stream, const streamDataT* data );

		//Checks the stream for a timeout
		template<typename streamDataT>
		mavel_data_stream_states stream_check( mavel_data_stream<streamDataT> &stream );
};
