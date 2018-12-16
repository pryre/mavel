#include <mavel/mavel.h>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <eigen3/Eigen/Dense>
#include <pid_controller_lib/pidController.h>

#include <string>
#include <math.h>

#define GRAVITY		9.80665f
#define SIGMA		0.000001f
#define MIN_DIST	0.01f

Mavel::Mavel() :
	nh_(),
	nhp_( "~" ),
	control_started_(false),
	control_fatal_(false),
	//param_got_valid_pos_(false),
	//param_got_valid_traj_(false),
	param_got_valid_tri_(false),
	//integrator_body_rate_z_( 0.0 ),
	ref_path_(nhp_),
	controller_pos_x_(ros::NodeHandle(nhp_, "control/pos/x")),
	controller_pos_y_(ros::NodeHandle(nhp_, "control/pos/y")),
	controller_pos_z_(ros::NodeHandle(nhp_, "control/pos/z")),
	controller_vel_x_(ros::NodeHandle(nhp_, "control/vel/x")),
	controller_vel_y_(ros::NodeHandle(nhp_, "control/vel/y")),
	controller_vel_z_(ros::NodeHandle(nhp_, "control/vel/z")) {

	nhp_.param( "control_rate", param_rate_control_, 50.0 );

	nhp_.param( "uav_mass", param_uav_mass_, 0.0 );

	nhp_.param( "tilt_max", param_tilt_max_, 0.39 );

	nhp_.param( "throttle/min", param_throttle_min_, 0.0 );
	nhp_.param( "throttle/mid", param_throttle_mid_, 0.5 );
	nhp_.param( "throttle/max", param_throttle_max_, 0.9 );

	nhp_.param( "failsafe_land_vel", param_land_vel_, -0.2 );
	nhp_.param( "failsafe_output_on_fatal", param_output_low_on_fatal_, false );
	nhp_.param( "allow_timeout_position", param_allow_timeout_position_, false );

	nhp_.param( "control_frame", param_control_frame_id_, std::string("map") );
	ref_path_.set_frame_id(param_control_frame_id_);

	//Data streams
	nhp_.param( "min_rate/state/odometry", param_stream_min_rate_state_odometry_, 20.0 );
	nhp_.param( "min_rate/state/mav_state", param_stream_min_rate_state_mav_, 0.2 );
	nhp_.param( "min_rate/reference/triplet", param_stream_min_rate_reference_triplet_, 20.0 );
	//nhp_.param( "min_rate/reference/trajectory", param_stream_min_rate_reference_trajectory_, 20.0 );
	//nhp_.param( "min_rate/reference/position", param_stream_min_rate_reference_position_, 5.0 );
	//nhp_.param( "min_rate/reference/velocity", param_stream_min_rate_reference_velocity_, 20.0 );
	//nhp_.param( "min_rate/reference/acceleration", param_stream_min_rate_reference_acceleration_, 40.0 );

	/*
	ROS_INFO("Minimum Rate (odom): %0.4f", param_stream_min_rate_state_odometry_);
	ROS_INFO("Minimum Rate (mav): %0.4f", param_stream_min_rate_state_mav_);
	ROS_INFO("Minimum Rate (traj): %0.4f", param_stream_min_rate_reference_trajectory_);
	ROS_INFO("Minimum Rate (pos): %0.4f", param_stream_min_rate_reference_position_);
	ROS_INFO("Minimum Rate (vel): %0.4f", param_stream_min_rate_reference_velocity_);
	ROS_INFO("Minimum Rate (accel): %0.4f", param_stream_min_rate_reference_acceleration_);
	*/
	//Sanity check some parameters
	if( ( param_land_vel_ >= 0.0) ) {
		 ROS_FATAL("Invalid parameter settings, quiting mavel");
		 ros::shutdown();
	}

	stream_state_odometry_ = stream_init<nav_msgs::Odometry>( param_stream_min_rate_state_odometry_, "state/odometry" );
	stream_state_mav_ = stream_init<mavros_msgs::State>( param_stream_min_rate_state_mav_, "state/mav_state" );
	stream_reference_triplet_ = stream_init<mavros_msgs::PositionTarget>( param_stream_min_rate_reference_triplet_, "reference/triplet" );
	//stream_reference_trajectory_ = stream_init<nav_msgs::Odometry>( param_stream_min_rate_reference_trajectory_, "reference/traj" );
	//stream_reference_position_ = stream_init<geometry_msgs::PoseStamped>( param_stream_min_rate_reference_position_, "reference/pose" );
	//stream_reference_velocity_ = stream_init<geometry_msgs::TwistStamped>( param_stream_min_rate_reference_velocity_, "reference/twist" );
	//stream_reference_acceleration_ = stream_init<geometry_msgs::AccelStamped>( param_stream_min_rate_reference_acceleration_, "reference/accel" );

	//Publishers
	pub_output_attitude_ = nhp_.advertise<mavros_msgs::AttitudeTarget>( "command/attitude", 100 );
	pub_output_wrench_ = nhp_.advertise<geometry_msgs::WrenchStamped>( "feedback/wrench", 100 );
	pub_output_acceleration_ = nhp_.advertise<geometry_msgs::AccelStamped>( "feedback/accel", 100 );
	pub_output_velocity_ = nhp_.advertise<geometry_msgs::TwistStamped>( "feedback/twist", 100 );
	pub_output_position_ = nhp_.advertise<geometry_msgs::PoseStamped>( "feedback/pose", 100 );
	pub_output_triplet_ = nhp_.advertise<mavros_msgs::PositionTarget>( "feedback/triplet", 100 );

	//Subscribers
	sub_state_odometry_ = nhp_.subscribe<nav_msgs::Odometry>( "state/odometry", 10, &Mavel::state_odometry_cb, this );
	sub_state_mav_ = nhp_.subscribe<mavros_msgs::State>( "state/mav_state", 10, &Mavel::state_mav_cb, this );
	//sub_reference_acceleration_ = nhp_.subscribe<geometry_msgs::AccelStamped>( "reference/accel", 10, &Mavel::reference_acceleration_cb, this );
	//sub_reference_velocity_ = nhp_.subscribe<geometry_msgs::TwistStamped>( "reference/twist", 10, &Mavel::reference_velocity_cb, this );
	//sub_reference_position_ = nhp_.subscribe<geometry_msgs::PoseStamped>( "reference/pose", 10, &Mavel::reference_position_cb, this );
	//sub_reference_trajectory_ = nhp_.subscribe<nav_msgs::Odometry>( "reference/traj", 10, &Mavel::reference_trajectory_cb, this );
	sub_reference_triplet_ = nhp_.subscribe<mavros_msgs::PositionTarget>( "reference/triplet", 10, &Mavel::reference_triplet_cb, this );

	//Wait for streams before starting
	ROS_INFO("Mavel ready, waiting for control inputs...");

	//Timers for controllers
	timer_controller_ = nhp_.createTimer( ros::Duration( 1.0 / param_rate_control_ ), &Mavel::controller_cb, this );
}

Mavel::~Mavel() {
}

//==-- Process:
//Position Control
//	Perform timing checks
//	Check active
//	Check reference data is fresh
//	Check setpoint data is fresh
//	Step PID controller
//	Save data for feedback

//Velocity Control
//	Perform timing checks
//	Check active
//	Check reference data is fresh
//	Check setpoint data is fresh
//	Step PID controller
//	Save data for feedback

//Acceleration Control
//	Perform timing checks
//	Check setpoint data is fresh
//	Calculate thrust vector

//Control Output
//	Check whether orientation or body rate should be used (and integrate accel body rate if necessary)
//	Do input masking as appropriate
//	Send attitude target
//	Send position, velocity, and acceleration feedback where relevant

void Mavel::state_odometry_cb( const nav_msgs::Odometry msg_in ) {
	stream_update( stream_state_odometry_, &msg_in );
}

void Mavel::state_mav_cb( const mavros_msgs::State msg_in ) {
	stream_update( stream_state_mav_, &msg_in );
}

void Mavel::reference_triplet_cb( const mavros_msgs::PositionTarget msg_in ) {
	//Check to make sure we're in the right reference frame
	if(msg_in.coordinate_frame == msg_in.FRAME_LOCAL_NED) {
		if( (msg_in.type_mask == TRIPLET_SETP_POS) ||
			(msg_in.type_mask == TRIPLET_SETP_VEL) ||
			(msg_in.type_mask == TRIPLET_SETP_ACC) ||
			(msg_in.type_mask == TRIPLET_TRAJ_PVEL) ||
			(msg_in.type_mask == TRIPLET_TRAJ_FULL) ) {

			stream_update( stream_reference_triplet_, &msg_in );

			if( (msg_in.header.stamp > ros::Time(0)) &&
				( (msg_in.type_mask == TRIPLET_SETP_POS) || (msg_in.type_mask == TRIPLET_TRAJ_PVEL) || (msg_in.type_mask == TRIPLET_TRAJ_FULL) ) ) {

				if(param_allow_timeout_position_ && !param_got_valid_tri_) {
					ROS_INFO("Mavel stream fallback initialized on: %s", stream_reference_triplet_.stream_topic.c_str() );
				}

				param_got_valid_tri_ = true;
			} else {
				param_got_valid_tri_ = false;	//XXX: Need to reset this, otherwise we might hang onto a vel by accident
			}
		} else {
			ROS_WARN_THROTTLE(2.0, "Unsupported triplet type mask: %i", msg_in.type_mask);
		}
	} else {
		ROS_WARN_THROTTLE(2.0, "Unsupported triplet coordinate frame: %i", msg_in.coordinate_frame);
	}
}
/*
void Mavel::reference_trajectory_cb( const nav_msgs::Odometry msg_in ) {
	stream_update( stream_reference_trajectory_, &msg_in );

	//Just check if the message header is OK
	if(msg_in.header.stamp > ros::Time(0)) {
		if(param_allow_timeout_position_ && !param_got_valid_traj_) {
			ROS_INFO("Mavel stream fallback initialized on: %s", stream_reference_trajectory_.stream_topic.c_str() );
		}

		param_got_valid_traj_ = true;
	}
}

void Mavel::reference_position_cb( const geometry_msgs::PoseStamped msg_in ) {
	stream_update( stream_reference_position_, &msg_in );

	//Just check if the message header is OK
	if(msg_in.header.stamp > ros::Time(0)) {
		if(param_allow_timeout_position_ && !param_got_valid_pos_) {
			ROS_INFO("Mavel stream fallback initialized on: %s", stream_reference_position_.stream_topic.c_str() );
		}

		param_got_valid_pos_ = true;
	}
}

void Mavel::reference_velocity_cb( const geometry_msgs::TwistStamped msg_in ) {
	stream_update( stream_reference_velocity_, &msg_in );
}

void Mavel::reference_acceleration_cb( const geometry_msgs::AccelStamped msg_in ) {
	stream_update( stream_reference_acceleration_, &msg_in );
}
*/
bool Mavel::flight_ready( const ros::Time check_time ) {
	bool ready = false;

	if( stream_check( stream_state_mav_, check_time ) == HEALTH_OK ) {
		ready = stream_state_mav_.data.armed && (stream_state_mav_.data.mode == "OFFBOARD");
	}

	return ready;
}

template<typename streamDataT>
mavel_data_stream<streamDataT> Mavel::stream_init( const double min_rate, const std::string topic ) {
	mavel_data_stream<streamDataT> stream;

	stream.state = HEALTH_UNKNOWN;

	stream.count = 0;
	stream.timeout = ros::Duration( ros::Rate(min_rate) );
	stream.stream_count = floor( 2 * min_rate );
	stream.stream_count = (stream.stream_count < 2) ? 2 : stream.stream_count;	//Account for slow stream like state

	stream.data.header.stamp = ros::Time( 0 );
	stream.stream_topic = topic;

	return stream;
}

template<typename streamDataT>
void Mavel::stream_update( mavel_data_stream<streamDataT> &stream, const streamDataT* data ) {
	stream.data = *data;

	//Only increment the counter if the stream data is fresh
	if( ( ros::Time::now() - stream.data.header.stamp ) < stream.timeout )
		stream.count++;
}

template<typename streamDataT>
mavel_data_stream_states Mavel::stream_check( mavel_data_stream<streamDataT> &stream, const ros::Time tc ) {
	//If the latest data in the stream is older than the timeout
	if( ( tc - stream.data.header.stamp ) > stream.timeout ) {
		//If the stream was previously OK, then send warning
		if( stream.state == HEALTH_OK ) {
			stream.state = HEALTH_TIMEOUT;
			ROS_WARN("Mavel stream timeout on: %s", stream.stream_topic.c_str() );
		}

		//Reset the stream counter
		stream.count = 0;
	} else if( ( stream.count > stream.stream_count ) && ( stream.state != HEALTH_OK ) ) {
		//If we have established a stream, and it wasn't already OK
		stream.state = HEALTH_OK;
		ROS_INFO("Mavel stream connected on: %s", stream.stream_topic.c_str() );
	}

	return stream.state;
}

void Mavel::controller_cb( const ros::TimerEvent& te ) {
	bool stream_state_odom_ok = stream_check( stream_state_odometry_, te.current_real ) == HEALTH_OK;
	bool stream_ref_tri_ok = stream_check( stream_reference_triplet_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_traj_ok = stream_check( stream_reference_trajectory_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_pos_ok = stream_check( stream_reference_position_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_vel_ok = stream_check( stream_reference_velocity_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_accel_ok = stream_check( stream_reference_acceleration_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_tpos_ok = param_allow_timeout_position_ && (param_got_valid_pos_ || param_got_valid_traj_ || param_got_valid_tri_);
	//bool stream_ref_path_ok = ref_path_.has_valid_path() || ref_path_.has_valid_fallback(); //Don't use a real stream, as it's more of a once off
	bool stream_ref_path_ok = ref_path_.has_reference(te.current_real); //Don't use a real stream, as it's more of a once off

	bool reference_ok = //( stream_ref_accel_ok ) ||
						( stream_state_odom_ok && ( //stream_ref_vel_ok ||
													//stream_ref_pos_ok ||
													//stream_ref_traj_ok ||
													//stream_ref_tpos_ok ||
													stream_ref_path_ok ||
													stream_ref_tri_ok ) );

	bool state_ok = stream_state_odom_ok;
	bool arm_ok = flight_ready(te.current_real);

	mavros_msgs::AttitudeTarget msg_out;
	msg_out.orientation.w = 1.0;	//Just to make sure the quaternion is initialized correctly

	if( !control_fatal_ ) {
		if( !control_started_ ) {
			if( state_ok && reference_ok ) {
				ROS_INFO_THROTTLE( 2.0, "Mavel ready, waiting for mav to be armed");

				if( arm_ok ) {
					control_started_ = true;
					ROS_INFO("Mav armed starting position control!");
				}
			}

			do_failsafe( te, msg_out );
		} else {
			if( arm_ok && state_ok ) {
				do_control( te, msg_out );
			} else {
				control_fatal_ = true;
			}
		}
	} else {
		ROS_ERROR_THROTTLE( 2.0, "[PANIC] Reference or arming error! Failsafe enabled!");
		do_failsafe( te, msg_out );
	}

	if( (!control_fatal_) || (!control_started_ || param_output_low_on_fatal_) ) {
		//Add in headers for attitude taget
		msg_out.header.frame_id = param_control_frame_id_;
		msg_out.header.stamp = te.current_real;

		pub_output_attitude_.publish( msg_out );
	}
}

void Mavel::do_control( const ros::TimerEvent& te, mavros_msgs::AttitudeTarget &goal_att ) {
	bool do_control_pos = false;
	bool do_control_vel = false;
	bool do_control_accel = false;

	mavros_msgs::PositionTarget goal_tri;

	double dt = (te.current_real - te.last_real).toSec();

	//Don't use a real stream for high-level, as it's more of a once off
	//bool stream_ref_path_ok = ref_path_.has_valid_path() || ref_path_.has_valid_fallback();
	//bool stream_ref_tpos_ok = param_allow_timeout_position_ && (param_got_valid_pos_ || param_got_valid_traj_ || param_got_valid_tri_);

	//Real input streams
	bool stream_ref_tri_ok = stream_check( stream_reference_triplet_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_traj_ok = stream_check( stream_reference_trajectory_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_pos_ok = stream_check( stream_reference_position_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_vel_ok = stream_check( stream_reference_velocity_, te.current_real ) == HEALTH_OK;
	//bool stream_ref_accel_ok = stream_check( stream_reference_acceleration_, te.current_real ) == HEALTH_OK;

	geometry_msgs::Point l_pos_ref;
	l_pos_ref.x = 0.0;
	l_pos_ref.y = 0.0;
	l_pos_ref.z = 0.0;

	geometry_msgs::Quaternion q_rot_ref;
	q_rot_ref.w = 1.0;
	q_rot_ref.x = 0.0;
	q_rot_ref.y = 0.0;
	q_rot_ref.z = 0.0;

	geometry_msgs::Vector3 l_vel_ref;
	l_vel_ref.x = 0.0;
	l_vel_ref.y = 0.0;
	l_vel_ref.z = 0.0;

	double r_rot_ref = 0.0;

	geometry_msgs::Vector3 l_acc_ref;
	l_acc_ref.x = 0.0;
	l_acc_ref.y = 0.0;
	l_acc_ref.z = 0.0;

	geometry_msgs::Vector3 thrust_ref;
	thrust_ref.x = 0.0;
	thrust_ref.y = 0.0;
	thrust_ref.z = 0.0;

	tf2::Transform state_tf;
	tf2::Vector3 state_vel;

	tf2::Vector3 tmp_pos( stream_state_odometry_.data.pose.pose.position.x,
						  stream_state_odometry_.data.pose.pose.position.y,
						  stream_state_odometry_.data.pose.pose.position.z );

	tf2::Quaternion tmp_q( stream_state_odometry_.data.pose.pose.orientation.x,
						   stream_state_odometry_.data.pose.pose.orientation.y,
						   stream_state_odometry_.data.pose.pose.orientation.z,
						   stream_state_odometry_.data.pose.pose.orientation.w );

	state_tf = tf2::Transform( tmp_q, tmp_pos );

	tf2::Quaternion rot_vel;
	rot_vel = ( tmp_q * tf2::Quaternion( stream_state_odometry_.data.twist.twist.linear.x,
									   stream_state_odometry_.data.twist.twist.linear.y,
									   stream_state_odometry_.data.twist.twist.linear.z,
									   0.0 ) ) * tmp_q.inverse();

	state_vel = tf2::Vector3( rot_vel.getX(), rot_vel.getY(), rot_vel.getZ() );

	/*
	if( stream_ref_accel_ok ) {
		goal_accel = stream_reference_acceleration_.data;
		do_control_accel = true;
	} else if( stream_ref_vel_ok ) {
		goal_vel = stream_reference_velocity_.data;

		do_control_vel = true;
		do_control_accel = true;
	} else if ( stream_ref_pos_ok ) {
		goal_pos = stream_reference_position_.data;

		do_control_pos = true;
		do_control_vel = true;
		do_control_accel = true;
	} else if ( stream_ref_tri_ok ) {
		if(stream_reference_triplet_.data.type_mask == TRIPLET_FULL_POS) {
			goal_pos.header = stream_reference_triplet_.data.header;

			goal_pos.pose.position = stream_reference_triplet_.data.position;
			tf2::Quaternion q(tf2::Vector3(0.0,0.0,1.0),stream_reference_triplet_.data.yaw);
			goal_pos.pose.orientation.w = q.getW();
			goal_pos.pose.orientation.x = q.getX();
			goal_pos.pose.orientation.y = q.getY();
			goal_pos.pose.orientation.z = q.getZ();

			do_control_pos = true;
		} else if(stream_reference_triplet_.data.type_mask == TRIPLET_FULL_VEL) {
			goal_vel.header = stream_reference_triplet_.data.header;

			goal_vel.twist.linear = stream_reference_triplet_.data.velocity;
			goal_vel.twist.angular.z = stream_reference_triplet_.data.yaw_rate;
		} else if(stream_reference_triplet_.data.type_mask == TRIPLET_FULL_TRAJ) {
			goal_pos.header = stream_reference_triplet_.data.header;

			goal_pos.pose.position = stream_reference_triplet_.data.position;
			tf2::Quaternion q(tf2::Vector3(0.0,0.0,1.0),stream_reference_triplet_.data.yaw);
			goal_pos.pose.orientation.w = q.getW();
			goal_pos.pose.orientation.x = q.getX();
			goal_pos.pose.orientation.y = q.getY();
			goal_pos.pose.orientation.z = q.getZ();

			l_vel_ref = stream_reference_triplet_.data.velocity;
			//yr_ref = stream_reference_triplet_.data.yaw_rate;

			do_control_pos = true;
		} else {
			ROS_ERROR_THROTTLE(2.0, "Unexpected error handling position fallback");
		}

		do_control_vel = true;
		do_control_accel = true;
	} else if ( stream_ref_traj_ok ) {
		goal_pos.header.frame_id = param_control_frame_id_;
		goal_pos.header.stamp = te.current_real;

		tf2::Quaternion traj_q( stream_reference_trajectory_.data.pose.pose.orientation.x,
								stream_reference_trajectory_.data.pose.pose.orientation.y,
								stream_reference_trajectory_.data.pose.pose.orientation.z,
								stream_reference_trajectory_.data.pose.pose.orientation.w );

		tf2::Quaternion traj_rvel = ( traj_q * tf2::Quaternion( stream_reference_trajectory_.data.twist.twist.linear.x,
										   stream_reference_trajectory_.data.twist.twist.linear.y,
										   stream_reference_trajectory_.data.twist.twist.linear.z,
										   0.0 ) ) * traj_q.inverse();

		goal_pos.pose = stream_reference_trajectory_.data.pose.pose;
		l_vel_ref.x = traj_rvel.getX();
		l_vel_ref.y = traj_rvel.getY();
		l_vel_ref.z = traj_rvel.getZ();

		do_control_pos = true;
		do_control_vel = true;
		do_control_accel = true;
	} else if( stream_ref_tpos_ok ) {
		if(param_got_valid_pos_) {
			goal_pos = stream_reference_position_.data;
		} else if(param_got_valid_traj_) {	//param_got_valid_traj_ == true
			//But handle it as a pure position setpoint
			goal_pos.header = stream_reference_trajectory_.data.header;
			goal_pos.pose = stream_reference_trajectory_.data.pose.pose;
		} else if(param_got_valid_tri_) {
			goal_pos.header = stream_reference_triplet_.data.header;
			goal_pos.pose.position = stream_reference_triplet_.data.position;
			tf2::Quaternion q(tf2::Vector3(0.0,0.0,1.0),stream_reference_triplet_.data.yaw);
			goal_pos.pose.orientation.w = q.getW();
			goal_pos.pose.orientation.x = q.getX();
			goal_pos.pose.orientation.y = q.getY();
			goal_pos.pose.orientation.z = q.getZ();
		}else {
			ROS_ERROR_THROTTLE(2.0, "Unexpected error handling position fallback");
		}

		do_control_pos = true;
		do_control_vel = true;
		do_control_accel = true;
	} else if ( stream_ref_path_ok ) {
		goal_pos.header.frame_id = param_control_frame_id_;
		goal_pos.header.stamp = te.current_real;
		ref_path_.get_ref_state(goal_pos.pose, l_vel_ref, te.current_real);

		do_control_pos = true;
		do_control_vel = true;
		do_control_accel = true;
	} else {
		goal_vel.header.frame_id = param_control_frame_id_;
		goal_vel.header.stamp = te.current_real;
		goal_vel.twist.linear.z = param_land_vel_;

		do_control_vel = true;
		do_control_accel = true;

		ROS_ERROR_THROTTLE( 2.0, "[Timeout] Setpoint error! Emergency landing!");
	}
	*/

	//Allows the pathing to be overriden by the tri input
	if( stream_ref_tri_ok ) {
		goal_tri = stream_reference_triplet_.data;
	} else {
		if(ref_path_.has_reference(te.current_real) && !ref_path_.get_reference(goal_tri, te.current_real, stream_state_odometry_.data.pose.pose) ) {
			ROS_ERROR_THROTTLE( 2.0, "[Contrail] Have setpoint but unable to generate reference!");
		}
	}

	//As long as we have a valid input
	if( goal_tri.header.stamp > ros::Time(0) ) {
		if(goal_tri.type_mask == TRIPLET_TRAJ_FULL) {
			l_pos_ref = goal_tri.position;
			tf2::Quaternion q(tf2::Vector3(0.0,0.0,1.0),goal_tri.yaw);
			q_rot_ref.w = q.getW();
			q_rot_ref.x = q.getX();
			q_rot_ref.y = q.getY();
			q_rot_ref.z = q.getZ();

			l_vel_ref = goal_tri.velocity;
			r_rot_ref = goal_tri.yaw_rate;

			l_acc_ref = goal_tri.acceleration_or_force;

			do_control_pos = true;
			do_control_vel = true;
			do_control_accel = true;
		} else if(goal_tri.type_mask == TRIPLET_TRAJ_PVEL) {
			l_pos_ref = goal_tri.position;
			tf2::Quaternion q(tf2::Vector3(0.0,0.0,1.0),goal_tri.yaw);
			q_rot_ref.w = q.getW();
			q_rot_ref.x = q.getX();
			q_rot_ref.y = q.getY();
			q_rot_ref.z = q.getZ();

			l_vel_ref = goal_tri.velocity;
			r_rot_ref = goal_tri.yaw_rate;

			do_control_pos = true;
			do_control_vel = true;
			do_control_accel = true;
		} else if(goal_tri.type_mask == TRIPLET_SETP_POS) {
			l_pos_ref = goal_tri.position;
			tf2::Quaternion q(tf2::Vector3(0.0,0.0,1.0),goal_tri.yaw);
			q_rot_ref.w = q.getW();
			q_rot_ref.x = q.getX();
			q_rot_ref.y = q.getY();
			q_rot_ref.z = q.getZ();

			do_control_pos = true;
			do_control_vel = true;
			do_control_accel = true;
		} else if(stream_reference_triplet_.data.type_mask == TRIPLET_SETP_VEL) {
			l_vel_ref = goal_tri.velocity;
			r_rot_ref = goal_tri.yaw_rate;

			do_control_vel = true;
			do_control_accel = true;
		} else if(stream_reference_triplet_.data.type_mask == TRIPLET_SETP_ACC) {
			l_acc_ref = goal_tri.acceleration_or_force;
			r_rot_ref = goal_tri.yaw_rate;

			do_control_accel = true;
		} else {
			ROS_ERROR_THROTTLE(2.0, "[Mavel] Unexpected error handling triplet: triplet had unexpected layout");
		}
	}

	//If we have no form of control input
	if(!do_control_accel) {
		goal_tri.header.frame_id = param_control_frame_id_;
		goal_tri.header.stamp = te.current_real;
		goal_tri.type_mask = TRIPLET_SETP_VEL;
		goal_tri.velocity.x = 0.0;
		goal_tri.velocity.y = 0.0;
		goal_tri.velocity.z = param_land_vel_;
		goal_tri.yaw_rate = 0.0;

		l_vel_ref = goal_tri.velocity;
		r_rot_ref = goal_tri.yaw_rate;

		do_control_vel = true;
		do_control_accel = true;

		ROS_ERROR_THROTTLE( 2.0, "[Timeout] Setpoint error! Emergency landing!");
	}

	//Position/Trajectory/Path Control
	if( do_control_pos ) {
		l_vel_ref.x += controller_pos_x_.step( dt, l_pos_ref.x, state_tf.getOrigin().getX() );
		l_vel_ref.y += controller_pos_y_.step( dt, l_pos_ref.y, state_tf.getOrigin().getY() );
		l_vel_ref.z += controller_pos_z_.step( dt, l_pos_ref.z, state_tf.getOrigin().getZ() );
	} else {
		//Prevent PID wind-up
		controller_pos_x_.reset( state_tf.getOrigin().getX() );
		controller_pos_y_.reset( state_tf.getOrigin().getY() );
		controller_pos_z_.reset( state_tf.getOrigin().getZ() );
	}

	//Velocity Controller
	if( do_control_vel ) {
		thrust_ref.x += controller_vel_x_.step( dt, l_vel_ref.x, state_vel.getX() );
		thrust_ref.y += controller_vel_y_.step( dt, l_vel_ref.y, state_vel.getY() );
		thrust_ref.z += controller_vel_z_.step( dt, l_vel_ref.z, state_vel.getZ() );
	} else {
		//Prevent PID wind-up
		controller_vel_x_.reset( state_vel.getX() );
		controller_vel_y_.reset( state_vel.getY() );
		controller_vel_z_.reset( state_vel.getZ() );
	}

	tf2::Vector3 T(0.0, 0.0, 0.0);
	double est_force_max = 0.0;

	if( do_control_accel ) {
		tf2::Vector3 l_acc_add(0.0, 0.0, 0.0);

		if(param_uav_mass_ > 0.0) {
			double est_throttle_hover = param_throttle_mid_ + controller_vel_z_.getOutputIterm();
			double est_force_hover = param_uav_mass_ * GRAVITY;
			est_force_max = est_force_hover / est_throttle_hover;
			double est_accel_max = est_force_max / param_uav_mass_;

			l_acc_add.setX(param_uav_mass_ * (l_acc_ref.x / est_accel_max) );
			l_acc_add.setY(param_uav_mass_ * (l_acc_ref.y / est_accel_max) );
			l_acc_add.setZ(param_uav_mass_ * (l_acc_ref.z / est_accel_max) );
		} else {
			ROS_WARN_ONCE("[Mavel] Paremter mass <= 0, not using acceleration reference");
		}

		//Calculate goal accelerations
		T.setX(thrust_ref.x);
		T.setY(thrust_ref.y);
		T.setZ(thrust_ref.z + param_throttle_mid_);
		T += l_acc_add;

		//Thrust Calculation
		tf2::Vector3 gThrust = T;
		tf2::Vector3 xyThrust(T.getX(), T.getX(), 0.0);
		tf2::Quaternion goalThrustRotation(0.0, 0.0, 0.0, 1.0);

		//If xy thrust vector length greater than 0.01
		if( xyThrust.length() > MIN_DIST ) {
			//Limit horizontal thrust by z thrust
			double thrust_xy_max = gThrust.getZ() * std::tan( param_tilt_max_ );

			//If thrust_sp_xy_len > thrust_xy_max
			if( xyThrust.length() > thrust_xy_max ) {
				//Scale the XY thrust setpoint down
				double k = thrust_xy_max / xyThrust.length();
				gThrust.setX( k*gThrust.getX() );
				gThrust.setY( k*gThrust.getY() );
				xyThrust.setX( gThrust.getX() );
				xyThrust.setY( gThrust.getY() );
			}
		}

		//If thrust_abs > thr_max
		if( gThrust.length() > param_throttle_max_ ) {
			//If thrust_z larger than thr_max, limit throttle, set xy to 0
			if( gThrust.getZ() > param_throttle_max_ ) {
				gThrust.setX( 0.0 );
				gThrust.setY( 0.0 );
				gThrust.setZ( param_throttle_max_ );
				ROS_WARN_THROTTLE( 1.0, "Too much thrust, scaling Z" );
			} else { //The XY thrust is the cause of the over-thrust, so scale down
				double thrust_xy_max = sqrtf( ( param_throttle_max_ * param_throttle_max_ ) - ( gThrust.getZ() * gThrust.getZ() ) );
				double k = thrust_xy_max / xyThrust.length();
				gThrust.setX( k * gThrust.getX() );
				gThrust.setY( k * gThrust.getY() );

				ROS_WARN_THROTTLE( 1.0, "Too much thrust, scaling XY" );
			}
		}

		//Create body_x, body_y, body_z
		tf2::Vector3 thrust_x;
		tf2::Vector3 thrust_y;
		tf2::Vector3 thrust_z;

		//thrust_abs > SIGMA
		if( gThrust.length() > SIGMA ) {
			//Get the direction of the thrust vector (and rotate to the body frame)
			thrust_z = gThrust.normalized();
		} else {
			//No thrust commanded, align with straight up
			thrust_z.setZ( 1.0 );
			//ROS_INFO("No thrust command, keeping zero Z orientation!");
		}

		double roll_c, pitch_c, yaw_c;

		if( do_control_pos ) {
			tf2::Quaternion q_c( q_rot_ref.x,
								 q_rot_ref.y,
								 q_rot_ref.z,
								 q_rot_ref.w );

			tf2::Matrix3x3( q_c ).getRPY( roll_c, pitch_c, yaw_c );
		}

		//Check to make sure the thrust vector is in the Z axis, or on the XY plane
		if ( fabs( thrust_z.getZ() ) > SIGMA ) {
			//Get a vector that aligns the Y axis with goal yaw
			tf2::Vector3 yaw_r( -sin( yaw_c ), cos( yaw_c ), 0.0 );
			//Get the orthagonal vector to that (which will have correct pitch)
			thrust_x = yaw_r.cross( thrust_z );

			//keep nose to front while inverted upside down
			if (thrust_z.getZ() < 0.0)
				thrust_x = -thrust_x;

			thrust_x.normalize();
		} else {
			// desired thrust is in XY plane, set X downside to construct correct matrix,
			// but yaw component will not be used actually
			thrust_x.setZ( 1.0 );
			ROS_INFO( "No thrust command, keeping zero XY orientation!" );
		}

		//Align the Y axis to be orthoganal with XZ plane
		thrust_y = thrust_z.cross( thrust_x );

		//Get rotation matrix
		tf2::Matrix3x3 R( thrust_x.getX(),
						  thrust_y.getX(),
						  thrust_z.getX(),
						  thrust_x.getY(),
						  thrust_y.getY(),
						  thrust_z.getY(),
						  thrust_x.getZ(),
						  thrust_y.getZ(),
						  thrust_z.getZ() );

		//Get quaternion from R
		R.getRotation( goalThrustRotation );

		goal_att.orientation.w = goalThrustRotation.getW();
		goal_att.orientation.x = goalThrustRotation.getX();
		goal_att.orientation.y = goalThrustRotation.getY();
		goal_att.orientation.z = goalThrustRotation.getZ();

		goal_att.thrust = gThrust.length();
	} else {
		//Prevent body z integrator wind-up
		//integrator_body_rate_z_ = 0.0;
	}

	//Do orientation control
	if( do_control_pos ) {
		/*
		//Make sure to add in the rate reference if we have it
		if(goal_tri.type_mask & TRIPLET_FULL_TRAJ) {
			goal_att.body_rate.z = goal_tri.yaw_rate;
			goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE;
		} else {
			goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE | goal_att.IGNORE_YAW_RATE;
		}
		*/
		goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE | goal_att.IGNORE_YAW_RATE;
	} else if( do_control_vel ) {
		goal_att.body_rate.z = r_rot_ref;//stream_reference_velocity_.data.twist.angular.z;
		goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE;
	} else if ( do_control_accel ) {
		//integrator_body_rate_z_ += dt * stream_reference_acceleration_.data.accel.angular.z;
		//goal_att.body_rate.z = integrator_body_rate_z_;
		goal_att.body_rate.z = r_rot_ref;
		goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE;
	}

	//Minimum throttle override
	if( goal_att.thrust < param_throttle_min_ )
		goal_att.thrust = param_throttle_min_;


	//Handle the control feedback
	if(do_control_accel) {
		geometry_msgs::WrenchStamped goal_wrench;

		goal_wrench.header = goal_tri.header;
		goal_wrench.wrench.force.x = T.getX() * est_force_max;
		goal_wrench.wrench.force.y = T.getY() * est_force_max;
		goal_wrench.wrench.force.z = T.getZ() * est_force_max;

		pub_output_wrench_.publish( goal_wrench );
		pub_output_triplet_.publish( goal_tri );
	}

	if( do_control_accel ) {
		geometry_msgs::AccelStamped goal_acc;

		goal_acc.header = goal_tri.header;
		goal_acc.accel.linear = l_acc_ref;
		goal_acc.accel.angular.x = 0.0;
		goal_acc.accel.angular.y = 0.0;
		goal_acc.accel.angular.z = 0.0;

		pub_output_acceleration_.publish( goal_acc );
	}

	if( do_control_vel ) {
		geometry_msgs::TwistStamped goal_vel;

		goal_vel.header = goal_tri.header;
		goal_vel.twist.linear = l_vel_ref;
		goal_vel.twist.angular.x = 0.0;
		goal_vel.twist.angular.y = 0.0;
		goal_vel.twist.angular.z = r_rot_ref;

		pub_output_velocity_.publish( goal_vel );
	}

	if( do_control_pos ) {
		geometry_msgs::PoseStamped goal_pos;

		goal_pos.header = goal_tri.header;
		goal_pos.pose.position = l_pos_ref;
		goal_pos.pose.orientation = q_rot_ref;

		pub_output_position_.publish( goal_pos );
	}
}

void Mavel::do_failsafe( const ros::TimerEvent& te, mavros_msgs::AttitudeTarget &goal_att ) {
	//Prevent PID wind-up
	controller_pos_x_.reset( 0.0 );
	controller_pos_y_.reset( 0.0 );
	controller_pos_z_.reset( 0.0 );
	controller_vel_x_.reset( 0.0 );
	controller_vel_y_.reset( 0.0 );
	controller_vel_z_.reset( 0.0 );
	//integrator_body_rate_z_ = 0.0;

	goal_att.orientation.w = 1.0;
	goal_att.orientation.x = 0.0;
	goal_att.orientation.y = 0.0;
	goal_att.orientation.z = 0.0;
	goal_att.body_rate.x = 0.0;
	goal_att.body_rate.y = 0.0;
	goal_att.body_rate.z = 0.0;
	goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE | goal_att.IGNORE_YAW_RATE | goal_att.IGNORE_ATTITUDE;
	goal_att.thrust = 0.0;
}
