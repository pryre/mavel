#pragma once

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <dynamic_reconfigure/server.h>
#include <mavel/MavelParamsConfig.h>

#include <eigen3/Eigen/Dense>
#include <contrail_manager/ContrailManager.h>
#include <pid_controller_lib/pidController.h>
#include <pid_controller_lib/pidTrackingController.h>

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

//Setup some defines to make handling triplet stream easier
//Full position and yaw goal
#define TRIPLET_SETP_POS 0b100111111000
//Full velocity and rate goal
#define TRIPLET_SETP_VEL 0b010111000111
//Full acceleration goal (with yaw rate)
#define TRIPLET_SETP_ACC 0b010000111111
//Full position and velocity goals
#define TRIPLET_TRAJ_PVEL 0b000111000000
//Full position and velocity goals
#define TRIPLET_TRAJ_FULL 0b000000000000
//Full position and velocity goal, but no rate
//#define TRIPLET_HALF_TRAJ ( (TRIPLET_FULL_POS & TRIPLET_FULL_VEL) | 0b100000000000 )

#define GRAVITY		9.80665f
#define SIGMA		0.000001f
#define MIN_DIST	0.01f

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

		ros::Publisher pub_output_triplet_;
		ros::Publisher pub_output_position_;
		ros::Publisher pub_output_velocity_;
		ros::Publisher pub_output_acceleration_;
		///ros::Publisher pub_output_wrench_;	//XXX: Currently unused
		ros::Publisher pub_output_attitude_;

		ros::Subscriber sub_state_mav_;
		ros::Subscriber sub_state_odometry_;
		ros::Subscriber sub_reference_triplet_;

		dynamic_reconfigure::Server<mavel::MavelParamsConfig> dyncfg_settings_;

		ros::Timer timer_controller_;
		bool control_started_;
		bool control_fatal_;

		double param_rate_control_;
		//double param_uav_mass_;	//XXX: Currently unused
		double param_tilt_max_;
		double param_throttle_min_;
		double param_throttle_mid_;
		double param_throttle_max_;
		double param_land_vel_;
		bool param_output_low_on_fatal_;
		bool param_allow_controller_reset_;
		bool param_use_tracking_pid_;

		bool param_allow_timeout_position_;
		bool param_got_valid_tri_;

		//Rate in Hz
		//Required stream count is derived as 2*rate
		double param_stream_min_rate_state_odometry_;
		double param_stream_min_rate_state_mav_;
		double param_stream_min_rate_reference_triplet_;

		mavel_data_stream<nav_msgs::Odometry> stream_state_odometry_;
		mavel_data_stream<mavros_msgs::State> stream_state_mav_;
		mavel_data_stream<mavros_msgs::PositionTarget> stream_reference_triplet_;

		ContrailManager ref_path_;
		pidController controller_pos_x_;
		pidController controller_pos_y_;
		pidController controller_pos_z_;
		pidController controller_vel_x_;
		pidController controller_vel_y_;
		pidController controller_vel_z_;
		pidTrackingController controller_track_x_;
		pidTrackingController controller_track_y_;
		pidTrackingController controller_track_z_;

		std::string param_control_frame_id_;

	public:
		Mavel( void );

		~Mavel( void );

		void shutdown( void );

		bool flight_ready( const ros::Time check_time );

	private:
		void callback_cfg_settings( mavel::MavelParamsConfig &config, uint32_t level );

		void state_odometry_cb( const nav_msgs::Odometry msg_in );
		void state_mav_cb( const mavros_msgs::State msg_in );

		void reference_triplet_cb( const mavros_msgs::PositionTarget msg_in );

		void reset_control_inputs( void );

		//Handles the controller loop
		void controller_cb( const ros::TimerEvent& timerCallback );
		void do_control( const ros::TimerEvent& timerCallback, mavros_msgs::AttitudeTarget &goal_att );
		void do_failsafe( const ros::TimerEvent& timerCallback, mavros_msgs::AttitudeTarget &goal_att );

		//Initializes the stream parameters
		template<typename streamDataT>
		mavel_data_stream<streamDataT> stream_init( const double min_rate, const std::string topic );

		//Updates the stream information that new data has been recieved
		//New data should have already been added into the stream
		template<typename streamDataT>
		void stream_update( mavel_data_stream<streamDataT> &stream, const streamDataT* data );

		//Checks the stream for a timeout
		template<typename streamDataT>
		mavel_data_stream_states stream_check( mavel_data_stream<streamDataT> &stream, const ros::Time tc );
};
