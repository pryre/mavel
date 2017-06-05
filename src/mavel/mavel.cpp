#include <mavel/mavel.h>
#include <pidController/pidController.h>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <string>

Mavel::Mavel() :
	nh_( "~" ),
	tfln_( tfbuffer_ ),
	param_rate_control_position_( 50.0f ),
	topic_output_attitude_( "~attitude_goal" ) {

	//Get parameters, or if not defined, use the defaults
	//nh_.param( "output_rate_control", param_output_rate_control_, param_output_rate_control_);
	//nh_.param( "topic_output_ping", topic_output_attitude_, topic_output_attitude_ );

	//Publishers for control feedback
	pub_output_attitude_ = nh_.advertise<mavros_msgs::AttitudeTarget>( topic_output_attitude_, 100 );
	pub_output_acceleration_ = nh_.advertise<geometry_msgs::AccelStamped>( topic_output_acceleration_, 100 );

	if( !param_external_acceleration_setpoint ) {
		pub_output_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>( topic_output_velocity_, 100 );

		if( !param_external_velocity_setpoint ) {
			pub_output_position_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_output_position_, 100 );
		}
	}

	//Subscribers for reference input
	if( !param_external_acceleration_setpoint ) {
		sub_reference_velocity_ = nh_.subscribe<geometry_msgs::TwistStamped>( topic_input_velocity_setpoint_, 100, &Mavel::setpoint_velocity_cb, this );

		if( !param_external_velocity_setpoint ) {
				sub_reference_position_ = nh_.subscribe<geometry_msgs::PoseStamped>( topic_input_position_setpoint_, 100, &Mavel::setpoint_position_cb, this );
		}
	}

	//Subscribers for setpoint input
	if( param_external_acceleration_setpoint ) {
		sub_setpoint_acceleration_ = nh_.subscribe<geometry_msgs::AccelStamped>( topic_input_acceleration_setpoint_, 100, &Mavel::setpoint_acceleration_cb, this );
	} else if( param_external_velocity_setpoint ) {
		sub_setpoint_velocity_ = nh_.subscribe<geometry_msgs::TwistStamped>( topic_input_velocity_setpoint_, 100, &Mavel::setpoint_velocity_cb, this );
	} else {
			sub_setpoint_position_ = nh_.subscribe<geometry_msgs::PoseStamped>( topic_input_position_setpoint_, 100, &Mavel::setpoint_position_cb, this );
	}

	//Timers for controllers
	timer_control_acceleration_ = nh_.createTimer( ros::Duration( param_rate_control_acceleration_ ), &Mavel::controller_acceleration_cb, this );

	if( !param_external_acceleration_setpoint ) {
		timer_control_velocity_ = nh_.createTimer( ros::Duration( param_rate_control_velocity_ ), &Mavel::controller_velocity_cb, this );

		if( !param_external_velocity_setpoint ) {
			timer_control_position_ = nh_.createTimer( ros::Duration( param_rate_control_position_ ), &Mavel::controller_position_cb, this );
		}
	}

	//================================================================================
	//XXX:Notes for later
	//================================================================================
	//	double dt = (timerCallback.current_real - timerCallback.last_real).toSec();
	//================================================================================
	//	geometry_msgs::TransformStamped transform = anything;
	//	geometry_msgs::PoseStamped pose;
	//	tf2::convert(transform, pose);
	//================================================================================

	ROS_INFO("Mavel setup and running!");
}

Mavel::~Mavel() {
	ROS_INFO("Mavel shutting down...");

	//TODO: Remember to free tf broadcaster
}

//==-- Process:
//Position Control
//	Perform timing checks
//	Check active
//	Check reference data is fresh
//	Check setpoint data is fresh
//	(TF stuff here)
//	Step PID controller
//	Save data for feedback

//Velocity Control
//	Perform timing checks
//	Check active
//	Check reference data is fresh
//	Check setpoint data is fresh
//	(TF stuff here)
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

void Mavel::reference_position_cb( const geometry_msgs::PoseStamped msg_in ) {
	stream_reference_position_.data = msg_in;
	stream_update( &stream_reference_position_ );
}

void Mavel::reference_velocity_cb( const geometry_msgs::TwistStamped msg_in ) {
	stream_reference_velocity_.data = msg_in;
	stream_update( &stream_reference_velocity_ );
}

void Mavel::setpoint_position_cb( const geometry_msgs::PoseStamped msg_in ) {
	stream_setpoint_position_.data = msg_in;
	stream_update( &stream_setpoint_position_ );
}

void Mavel::setpoint_velocity_cb( const geometry_msgs::TwistStamped msg_in ) {
	stream_setpoint_velocity_.data = msg_in;
	stream_update( &stream_setpoint_velocity_ );
}

void Mavel::setpoint_acceleration_cb( const geometry_msgs::AccelStamped msg_in ) {
	stream_setpoint_acceleration_.data = msg_in;
	stream_update( &stream_setpoint_acceleration_ );
}

void Mavel::controller_position_cb( const ros::TimerEvent& ) {
	if( ( stream_reference_position_.state == HEALTH_OK )  &&
		( stream_setpoint_position_.state == HEALTH_OK ) ) {

	}
}

void Mavel::controller_velocity_cb( const ros::TimerEvent& ) {
	if( ( stream_reference_velocity_.state == HEALTH_OK )  &&
		( stream_setpoint_velocity_.state == HEALTH_OK ) ) {

	}
}

void Mavel::controller_acceleration_cb( const ros::TimerEvent& ) {
	if( stream_setpoint_acceleration_.state == HEALTH_OK ) {

	}
}

void Mavel::param_pid_init( mavel_params_pid* pid ) {

}

template<typename streamDataT>
void Mavel::stream_init( mavel_data_stream<streamDataT>* stream ) {

}

template<typename streamDataT>
void Mavel::stream_update( mavel_data_stream<streamDataT>* stream ) {

}

template<typename streamDataT>
void Mavel::stream_check( mavel_data_stream<streamDataT>* stream, ros::Time time_now ) {

}

/*
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <string.h>
#include <math.h>

#include "pid.h"


#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f

//================================//
// Callback Functions             //
//================================//

geometry_msgs::TwistStamped goalVelocity;

void cmd_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	goalVelocity = *msg;
}

//================================//
// Main Function                  //
//================================//
int main(int argc, char **argv) {
	//================================//
	// Initialize node                //
	//================================//
	ros::init(argc, argv, "mavel" );
	ros::NodeHandle nh( ros::this_node::getName() );

	double commandTimeout = 0.1; //TODO: Params
	bool freshCommands = false;		//The thread will be locked to only sending safety values
	bool freshTransform = false;	//The thread will be locked to only sending safety values

	double param_loop_rate = 20.0;
	double param_trottle_threshold = 0.1;
	bool thr_threshold_passed = false;

	std::string param_cmd_vel = "cmd_vel";
	std::string param_cmd_att = "cmd_att";
	std::string param_tf_global = "world";
	std::string param_tf_body = "fcu/body";
	std::string param_tf_ref = "fcu/base_link";
	std::string param_tf_goal = "fcu/goal";

	double param_vel_xy_pid_p = 0.15;
	double param_vel_xy_pid_i = 0.01;
	double param_vel_xy_pid_d = 0.05;
	double param_vel_xy_pid_min = -1.0;
	double param_vel_xy_pid_max = 1.0;

	double param_tilt_max = 0.39;	//22.5 Deg

	double param_yaw_rate_ff = 10.0;

	double param_vel_z_pid_p = 0.1;
	double param_vel_z_pid_i = 0.1;
	double param_vel_z_pid_d = 0.05; //TODO: min max
	double param_vel_z_pid_min = -0.25;
	double param_vel_z_pid_max = 0.25;

	double param_throttle_min = 0.20;
	double param_throttle_mid = 0.5;
	double param_throttle_max = 0.90;

	//POS XY PID //================================================================
	ROS_INFO("[Input & Output Topics]");

	if( !nh.getParam( "velocity_input", param_cmd_vel ) ) {
		ROS_WARN( "No parameter set for \"velocity_input\", using: %s", param_cmd_vel.c_str() );
	}
	ROS_INFO( "Listening for velocity input: %s", param_cmd_vel.c_str() );

	if( !nh.getParam( "attitude_output", param_cmd_att ) ) {
		ROS_WARN( "No parameter set for \"attitude_output\", using: %s", param_cmd_att.c_str() );
	}
	ROS_INFO( "Broadcasting attitude goal: %s", param_cmd_att.c_str() );

	if( !nh.getParam( "global_frame", param_tf_global ) ) {
		ROS_WARN( "No parameter set for \"global_frame\", using: %s", param_tf_global.c_str() );
	}
	ROS_INFO( "TF global frame: %s", param_tf_global.c_str() );

	if( !nh.getParam( "body_frame", param_tf_body ) ) {
		ROS_WARN( "No parameter set for \"body_frame\", using: %s", param_tf_body.c_str() );
	}
	ROS_INFO( "TF body frame: %s", param_tf_body.c_str() );

	if( !nh.getParam( "body_reference", param_tf_ref ) ) {
		ROS_WARN( "No parameter set for \"body_reference\", using: %s", param_tf_ref.c_str() );
	}
	ROS_INFO( "TF body reference (0 rotation) frame: %s", param_tf_ref.c_str() );

	if( !nh.getParam( "body_goal", param_tf_goal ) ) {
		ROS_WARN( "No parameter set for \"body_goal\", using: %s", param_tf_goal.c_str() );
	}
	ROS_INFO( "TF body goal frame: %s", param_tf_goal.c_str() );

	//POS XY PID //================================================================
	ROS_INFO("[Velocity Controller]");

	if( !nh.getParam( "vel_xy_pid/p", param_vel_xy_pid_p) ) {
		ROS_WARN( "No parameter set for \"vel_xy_pid/p\", using: %0.2f", param_vel_xy_pid_p);
	}

	if( !nh.getParam( "vel_xy_pid/i", param_vel_xy_pid_i) ) {
		ROS_WARN( "No parameter set for \"vel_xy_pid/i\", using: %0.2f", param_vel_xy_pid_i);
	}

	if( !nh.getParam( "vel_xy_pid/d", param_vel_xy_pid_d) ) {
		ROS_WARN( "No parameter set for \"vel_xy_pid/d\", using: %0.2f", param_vel_xy_pid_d);
	}

	if( !nh.getParam( "vel_xy_pid/min", param_vel_xy_pid_min) ) {
		ROS_WARN( "No parameter set for \"vel_xy_pid/min\", using: %0.2f", param_vel_xy_pid_min);
	}

	if( !nh.getParam( "vel_xy_pid/max", param_vel_xy_pid_max) ) {
		ROS_WARN( "No parameter set for \"vel_xy_pid/max\", using: %0.2f", param_vel_xy_pid_max);
	}

	ROS_INFO( "Setting vel_xy_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", param_vel_xy_pid_p, param_vel_xy_pid_i, param_vel_xy_pid_d, param_vel_xy_pid_min, param_vel_xy_pid_max);

	//POS Z PID //================================================================

	if( !nh.getParam( "throttle/min", param_throttle_min) ) {
		ROS_WARN( "No parameter set for \"throttle/min\", using: %0.2f", param_throttle_min);
	}

	if( !nh.getParam( "throttle/mid", param_throttle_mid) ) {
		ROS_WARN( "No parameter set for \"throttle/mid\", using: %0.2f", param_throttle_mid);
	}

	if( !nh.getParam( "throttle/max", param_throttle_max) ) {
		ROS_WARN( "No parameter set for \"throttle/max\", using: %0.2f", param_throttle_max);
	}

	ROS_INFO( "Setting throttle to: [%0.2f, %0.2f, %0.2f]", param_throttle_min, param_throttle_mid, param_throttle_max);

	if( !nh.getParam( "vel_z_pid/p", param_vel_z_pid_p) ) {
		ROS_WARN( "No parameter set for \"pos_z_pid/p\", using: %0.2f", param_vel_z_pid_p);
	}

	if( !nh.getParam( "vel_z_pid/i", param_vel_z_pid_i) ) {
		ROS_WARN( "No parameter set for \"pos_z_pid/i\", using: %0.2f", param_vel_z_pid_i);
	}

	if( !nh.getParam( "vel_z_pid/d", param_vel_z_pid_d) ) {
		ROS_WARN( "No parameter set for \"pos_z_pid/d\", using: %0.2f", param_vel_z_pid_d);
	}

	param_vel_z_pid_min = -(param_throttle_mid - param_throttle_min);	//Set based on the throttle settings
	param_vel_z_pid_max = param_throttle_max - param_throttle_mid;		//Otherwise they would just cause additional pid windup


	ROS_INFO( "Setting vel_z_pid to: [%0.2f, %0.2f, %0.2f; %0.2f, %0.2f]", param_vel_z_pid_p, param_vel_z_pid_i, param_vel_z_pid_d, param_vel_z_pid_min, param_vel_z_pid_max);

	//PARAMS //================================================================
	ROS_INFO("[Misc. Parameters]");

	if( !nh.getParam( "rate_yaw/ff", param_yaw_rate_ff) ) {
		ROS_WARN( "No parameter set for \"rate_yaw/ff\", using: %0.2f", param_yaw_rate_ff);
	}
	ROS_INFO( "Setting yaw control feed forward to: %0.2f", param_yaw_rate_ff );

	if( !nh.getParam( "tilt_max", param_tilt_max) ) {
		ROS_WARN( "No parameter set for \"tilt_max\", using: %0.2f", param_tilt_max);
	}
	ROS_INFO( "Setting maximum roll/pitch angle (rad): %0.2f", param_tilt_max );

	if( !nh.getParam( "loop_rate", param_loop_rate) ) {
		ROS_WARN( "No parameter set for \"loop_rate\", using: %0.2f", param_loop_rate);
	}
	ROS_INFO( "Setting controller rate (Hz): %0.2f", param_loop_rate );


	ROS_INFO("[Parameters Loaded]");

	mavros_msgs::AttitudeTarget outputAttitude;
	outputAttitude.type_mask = outputAttitude.IGNORE_ROLL_RATE + outputAttitude.IGNORE_PITCH_RATE;	//We want to send yaw rate, but not other rates
	outputAttitude.header.frame_id = param_tf_goal;

	pid vel_x_pid( 1.0/param_loop_rate, param_vel_xy_pid_p, param_vel_xy_pid_i, param_vel_xy_pid_d, param_vel_xy_pid_min, param_vel_xy_pid_max );
	pid vel_y_pid( 1.0/param_loop_rate, param_vel_xy_pid_p, param_vel_xy_pid_i, param_vel_xy_pid_d, param_vel_xy_pid_min, param_vel_xy_pid_max );
	pid vel_z_pid( 1.0/param_loop_rate, param_vel_z_pid_p, param_vel_z_pid_i, param_vel_z_pid_d, param_vel_z_pid_min, param_vel_z_pid_max);

	//Subscribers
	ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		(param_cmd_vel, 10, cmd_vel_cb);

	ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
		(param_cmd_att, 10);

	//Initialize TF
	tf::TransformListener tfln;
	tf::TransformBroadcaster tfbr;

	ros::Rate rate(param_loop_rate);
	ros::spinOnce();

	//Sit and wait until the inputs are ready
	while( ros::ok() && ( !freshCommands || !freshTransform ) ) {
		freshCommands = commandTimeout > ( ros::Time::now() - goalVelocity.header.stamp ).toSec();
		freshTransform = tfln.waitForTransform( param_tf_global, param_tf_body, ros::Time::now(), ros::Duration(1.0) );

		ROS_INFO_THROTTLE( 2.0, "Waiting for inputs to become avaliable: [cmd: %d; tf: %d]", freshCommands, freshTransform );

		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO( "Inputs avaliable: [cmd: %d; tf: %d]", freshCommands, freshTransform );
	ROS_INFO( "Ready to begin broadcasting!" );

	//================================//
	// Main Loop                      //
	//================================//
	while( ros::ok() ) {
		tf::StampedTransform currentTransform;
		geometry_msgs::Twist bodyVelocity;
		geometry_msgs::Twist worldVelocity;
		tf::Transform goalTransform;
		double goalYawRate = 0.0;

		tf::Quaternion goalThrustRotation(0.0, 0.0, 0.0, 1.0);
		double goalThrust = param_throttle_min;

		//TODO: Should have something to set low throttle and no movement until a certain threshold is passed for the first time

		//Check to make sure there have been new commands
		if( commandTimeout < ( ros::Time::now() - goalVelocity.header.stamp ).toSec() ) {
			freshCommands = false;
			ROS_ERROR_ONCE( "[Timeout] No new velocity commands" );
		}

		if( freshTransform ) {
			try {
				//Get the latest pose of the fcu in the world
				tfln.lookupTransform(param_tf_global, param_tf_body, ros::Time(0), currentTransform);

				//Get the latest twist (velocity) estimate of the fcu relative to the world
				tfln.lookupTwist(param_tf_ref, param_tf_global, param_tf_ref, tf::Point(), param_tf_global, ros::Time(0), ros::Duration(0.5), worldVelocity);
			}

			catch (tf::TransformException ex) {
				ROS_ERROR( "%s",ex.what() );
				freshTransform = false;
			}

			//Only bother doing these steps if there is actually a new transform, as they will cause nan errors
			if( freshTransform ) {
				//Prepare the output rotation goal message
				//Need to use the current rotation to get the yaw rate to work (that it doesn't try to yaw if no commands are sent)
				geometry_msgs::Vector3 rot;
				tf::Matrix3x3(currentTransform.getRotation()).getRPY(rot.x, rot.y, rot.z);

				goalThrustRotation = currentTransform.getRotation();

				//Align velocity frame with the body frame
				tf::Vector3 vel;	//TODO: This probably only works when the UAV is right-side-up
				tf::vector3MsgToTF(worldVelocity.linear, vel);
				vel = tf::quatRotate(tf::createQuaternionFromYaw(-rot.z), vel);
				tf::vector3TFToMsg(vel, bodyVelocity.linear);

				ROS_DEBUG("Cur Vel: [%0.2f, %0.2f, %0.2f]", bodyVelocity.linear.x, bodyVelocity.linear.y, bodyVelocity.linear.z );
				ROS_DEBUG("Goal Vel: [%0.2f, %0.2f, %0.2f; %0.2f]", goalVelocity.twist.linear.x, goalVelocity.twist.linear.y, goalVelocity.twist.linear.z, goalVelocity.twist.angular.z);
			}
		} else {
			ROS_ERROR_ONCE( "[Timeout] No new transform information" );
		}

		//Main control loop
		//If inputs are lost at all, panic
		if( freshCommands && freshTransform && thr_threshold_passed ) {
			ROS_INFO_THROTTLE( 2.0, "Publishing attitude setpoints..." );

			//Calculate goal accelerations
			double Tx = vel_x_pid.step(goalVelocity.twist.linear.x, bodyVelocity.linear.x );
			double Ty = vel_y_pid.step(goalVelocity.twist.linear.y, bodyVelocity.linear.y );
			double Tz = vel_z_pid.step(goalVelocity.twist.linear.z, bodyVelocity.linear.z ) + param_throttle_mid; //TODO: Add in a hover approximation

			goalYawRate = goalVelocity.twist.angular.z;

			ROS_DEBUG("Goal Thrust: [%0.2f, %0.2f, %0.2f]", Tx, Ty, Tz);
			ROS_DEBUG("Goal Yaw Rate: [%0.2f]", goalYawRate);

			//Thrust Calculation
			tf::Vector3 gThrust(Tx, Ty, Tz);
			tf::Vector3 xyThrust(Tx, Ty, 0.0);

			//If xy thrust vector length greater than 0.01
			if( xyThrust.length() > MIN_DIST) {
				//Limit horizontal thrust by z thrust
				double thrust_xy_max = gThrust.getZ() * std::tan(param_tilt_max);

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
			if( gThrust.length() > param_throttle_max ) {
				//If thrust_z larger than thr_max, limit throttle, set xy to 0
				if( gThrust.getZ() > param_throttle_max ) {
					gThrust.setX( 0.0 );
					gThrust.setY( 0.0 );
					gThrust.setZ( param_throttle_max );
					ROS_WARN_THROTTLE(0.1, "Too much thrust, scaling Z");
				} else { //The XY thrust is the cause of the over-thrust, so scale down
					double thrust_xy_max = sqrtf( ( param_throttle_max * param_throttle_max ) - ( gThrust.getZ() * gThrust.getZ() ) );
					double k = thrust_xy_max / xyThrust.length();
					gThrust.setX( k * gThrust.getX() );
					gThrust.setY( k * gThrust.getY() );

					ROS_WARN_THROTTLE(0.1, "Too much thrust, scaling XY");
				}
			}

			//Create body_x, body_y, body_z
			tf::Vector3 body_x;
			tf::Vector3 body_y;
			tf::Vector3 body_z;

			//thrust_abs > SIGMA
			if( gThrust.length() > SIGMA) {
				//Get the direction of the thrust vector (and rotate to the body frame)
				body_z = gThrust.normalized();
				//body_z = tf::quatRotate(tf::createQuaternionFromYaw(goalHeading), body_z);

				ROS_DEBUG( "body_z: [%0.2f, %0.2f, %0.2f]", body_z.getX(), body_z.getY(), body_z.getZ() );
			} else {
				//No thrust commanded, align with straight up
				body_z.setZ( 1.0 );
				ROS_INFO("No thrust command, keeping zero Z orientation!");
			}

			//Check to make sure the thrust vector is in the Z axis, or on the XY plane
			if (fabs(body_z.getZ()) > SIGMA) {
				//Get a vector that aligns the Y axis with pi/2
				tf::Vector3 yaw_r( 0.0f, 1.0f, 0.0f );
				//Get the orthagonal vector to that (which will have correct pitch)
				body_x = yaw_r.cross(body_z);

				//keep nose to front while inverted upside down
				if (body_z.getZ() < 0.0f)
					body_x = -body_x;

				body_x.normalize();
			} else {
				// desired thrust is in XY plane, set X downside to construct correct matrix,
				// but yaw component will not be used actually
				body_x.setZ( 1.0 );
				ROS_INFO("No thrust command, keeping zero XY orientation!");
			}

			//Align the Y axis to be orthoganal with XZ plane
			body_y = body_z.cross(body_x);

			//Get rotation matrix
			tf::Matrix3x3 R(body_x.getX(),
							body_y.getX(),
							body_z.getX(),
							body_x.getY(),
							body_y.getY(),
							body_z.getY(),
							body_x.getZ(),
							body_y.getZ(),
							body_z.getZ());

			//Get quaternion from R
			R.getRotation( goalThrustRotation );
			goalThrust = gThrust.length();

			ROS_DEBUG( "Output Thrust: [%0.2f, %0.2f, %0.2f]", gThrust.getX(), gThrust.getY(), gThrust.getZ() );
			ROS_DEBUG( "Output Rotation:\n\tx: [%0.2f, %0.2f, %0.2f]\n\ty: [%0.2f, %0.2f, %0.2f]\n\tz: [%0.2f, %0.2f, %0.2f]",	body_x.getX(), body_x.getY(), body_x.getZ(),
																														body_y.getX(), body_y.getY(), body_y.getZ(),
																														body_z.getX(), body_z.getY(), body_z.getZ());
		} else {
			//Prevent PID windup
			vel_x_pid.reset();
			vel_y_pid.reset();
			vel_z_pid.reset();

			ROS_INFO_THROTTLE( 2.0, "Publishing to hold orientation, and low thrust..." );

			tf::Vector3 velCheck;
			tf::vector3MsgToTF( goalVelocity.twist.linear, velCheck);
			if(velCheck.length() > param_trottle_threshold) {	//TODO: Maybe should check for angular as well?
				thr_threshold_passed = true;
				ROS_INFO_ONCE("Velocity commands non-zero, activating flight");
			}
		}

		//TODO: When available: outputAttitude.body_rate = goalVelocity.twist.angular;
		outputAttitude.header.stamp = ros::Time::now();
		outputAttitude.header.seq++;
		tf::quaternionTFToMsg( goalThrustRotation, outputAttitude.orientation );
		outputAttitude.thrust = goalThrust;
		outputAttitude.body_rate.z = goalYawRate;

		goalTransform.setOrigin( tf::Vector3() );
		goalTransform.setRotation( goalThrustRotation );

		tfbr.sendTransform(tf::StampedTransform(goalTransform, ros::Time::now(), param_tf_ref, param_tf_goal));
		att_pub.publish(outputAttitude);

		//Sleep //================================================================
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

*/
