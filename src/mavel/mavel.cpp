#include <mavel/mavel.h>
#include <pidController/pidController.h>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <string>
#include <math>

#define SIGMA			0.000001f
#define MIN_DIST		0.01f

Mavel::Mavel() :
	nh_( "~" ),
	tfln_( tfbuffer_ ),
	integrator_body_rate_z_( 0.0f ) {

	nh_.param( "~control_rate", param_rate_control_, param_rate_control_ );

	nh_.param( "~tilt_max", param_tilt_max_, param_tilt_max_ );

	nh_.param( "~throttle/min", param_throttle_min_, param_throttle_min_ );
	nh_.param( "~throttle/mid", param_throttle_mid_, param_throttle_mid_ );
	nh_.param( "~throttle/max", param_throttle_max_, param_throttle_max_ );

	nh_.param( "~control_frame", param_control_frame_id_, param_control_frame_id_ );

	//PID Controllers
	param_pid_init( &param_pid_pos_x, "~pid/position/x" );
	param_pid_init( &param_pid_pos_y, "~pid/position/y" );
	param_pid_init( &param_pid_pos_z, "~pid/position/z" );
	param_pid_init( &param_pid_vel_x, "~pid/velocity/x" );
	param_pid_init( &param_pid_vel_y, "~pid/velocity/y" );
	param_pid_init( &param_pid_vel_z, "~pid/velocity/z" );

	controller_pos_x.setGains( param_pid_pos_x.kp, param_pid_pos_x.ki, param_pid_pos_x.kd, param_pid_pos_x.tau );
	controller_pos_x.setOutputMinMax( param_pid_pos_x.min, param_pid_pos_x.max );
	controller_pos_y.setGains( param_pid_pos_y.kp, param_pid_pos_y.ki, param_pid_pos_y.kd, param_pid_pos_y.tau );
	controller_pos_y.setOutputMinMax( param_pid_pos_y.min, param_pid_pos_y.max );
	controller_pos_z.setGains( param_pid_pos_z.kp, param_pid_pos_z.ki, param_pid_pos_z.kd, param_pid_pos_z.tau );
	controller_pos_z.setOutputMinMax( param_pid_pos_z.min, param_pid_pos_z.max );

	controller_vel_x.setGains( param_pid_vel_x.kp, param_pid_vel_x.ki, param_pid_vel_x.kd, param_pid_vel_x.tau );
	controller_vel_x.setOutputMinMax( param_pid_vel_x.min, param_pid_vel_x.max );
	controller_vel_y.setGains( param_pid_vel_y.kp, param_pid_vel_y.ki, param_pid_vel_y.kd, param_pid_vel_y.tau );
	controller_vel_y.setOutputMinMax( param_pid_vel_y.min, param_pid_vel_y.max );
	controller_vel_z.setGains( param_pid_vel_z.kp, param_pid_vel_z.ki, param_pid_vel_z.kd, param_pid_vel_z.tau );
	controller_vel_z.setOutputMinMax( param_pid_vel_z.min, param_pid_vel_z.max );

	//Data streams
	nh_.param( "~stream/min_rate/reference/position", param_stream_min_rate_position_reference_, param_stream_min_rate_position_reference_);
	nh_.param( "~stream/min_rate/reference/velocity", param_stream_min_rate_velocity_reference_, param_stream_min_rate_velocity_reference_);
	nh_.param( "~stream/min_rate/setpoint/position", param_stream_min_rate_position_setpoint_, param_stream_min_rate_position_setpoint_);
	nh_.param( "~stream/min_rate/setpoint/velocity", param_stream_min_rate_velocity_setpoint_, param_stream_min_rate_velocity_setpoint_);
	nh_.param( "~stream/min_rate/setpoint/acceleration", param_stream_min_rate_acceleration_setpoint_, param_stream_min_rate_acceleration_setpoint_);

	nh_.param( "~stream/topic/reference/position", topic_input_position_reference_, topic_input_position_reference_);
	nh_.param( "~stream/topic/reference/velocity", topic_input_velocity_reference_, topic_input_velocity_reference_);

	nh_.param( "~stream/topic/setpoint/position", topic_input_position_setpoint_, topic_input_position_setpoint_);
	nh_.param( "~stream/topic/setpoint/velocity", topic_input_velocity_setpoint_, topic_input_acceleration_setpoint_);
	nh_.param( "~stream/topic/setpoint/acceleration", topic_input_acceleration_setpoint_, topic_input_acceleration_setpoint_);

	nh_.param( "~stream/topic/feedback/position", topic_output_position_, topic_output_position_);
	nh_.param( "~stream/topic/feedback/velocity", topic_output_velocity_, topic_output_velocity_);
	nh_.param( "~stream/topic/feedback/acceleration", topic_output_acceleration_, topic_output_acceleration_);
	nh_.param( "~stream/topic/feedback/acceleration", topic_output_attitude_, topic_output_attitude_);

	stream_init( &stream_reference_position_, param_timeout_stream_reference_position_, topic_input_position_reference_ );
	stream_init( &stream_reference_velocity_, param_timeout_stream_reference_velocity_, topic_input_velocity_reference_ );
	stream_init( &stream_setpoint_position_, param_timeout_stream_setpoint_position_, topic_input_position_setpoint_ );
	stream_init( &stream_setpoint_velocity_, param_timeout_stream_setpoint_velocity_, topic_input_velocity_setpoint_ );
	stream_init( &stream_setpoint_acceleration_, param_timeout_stream_setpoint_acceleration_, topic_input_acceleration_setpoint_ );

	//Publishers for control feedback
	pub_output_attitude_ = nh_.advertise<mavros_msgs::AttitudeTarget>( topic_output_attitude_, 100 );
	pub_output_acceleration_ = nh_.advertise<geometry_msgs::AccelStamped>( topic_output_acceleration_, 100 );
	pub_output_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>( topic_output_velocity_, 100 );
	pub_output_position_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_output_position_, 100 );

	//Subscribers for reference input
	if( !param_external_acceleration_setpoint ) {
		sub_reference_velocity_ = nh_.subscribe<geometry_msgs::TwistStamped>( topic_input_velocity_reference_, 100, &Mavel::setpoint_velocity_cb, this );

		if( !param_external_velocity_setpoint ) {
				sub_reference_position_ = nh_.subscribe<geometry_msgs::PoseStamped>( topic_input_position_reference_, 100, &Mavel::setpoint_position_cb, this );
		}
	}

	//Subscribers for setpoint input
	sub_setpoint_acceleration_ = nh_.subscribe<geometry_msgs::AccelStamped>( topic_input_acceleration_setpoint_, 100, &Mavel::setpoint_acceleration_cb, this );
	sub_setpoint_velocity_ = nh_.subscribe<geometry_msgs::TwistStamped>( topic_input_velocity_setpoint_, 100, &Mavel::setpoint_velocity_cb, this );
	sub_setpoint_position_ = nh_.subscribe<geometry_msgs::PoseStamped>( topic_input_position_setpoint_, 100, &Mavel::setpoint_position_cb, this );

	//Timers for controllers
	timer_controller_ = nh_.createTimer( ros::Duration( param_rate_control_ ), &Mavel::controller_cb, this );

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

void Mavel::reference_position_cb( const geometry_msgs::PoseStamped msg_in ) {
	stream_update( &stream_reference_position_, &msg_in );
}

void Mavel::reference_velocity_cb( const geometry_msgs::TwistStamped msg_in ) {
	stream_update( &stream_reference_velocity_, &msg_in );
}

void Mavel::setpoint_position_cb( const geometry_msgs::PoseStamped msg_in ) {
	stream_update( &stream_setpoint_position_, &msg_in );
}

void Mavel::setpoint_velocity_cb( const geometry_msgs::TwistStamped msg_in ) {
	stream_update( &stream_setpoint_velocity_, &msg_in );
}

void Mavel::setpoint_acceleration_cb( const geometry_msgs::AccelStamped msg_in ) {
	stream_update( &stream_setpoint_acceleration_, &msg_in );
}

void Mavel::controller_cb( const ros::TimerEvent& ) {
	bool do_control_pos = false;
	bool do_control_vel = false;
	bool do_control_accel = false;

	bool stream_ref_pos_ok = stream_check( &stream_reference_position_ ) == HEALTH_OK;
	bool stream_ref_vel_ok = stream_check( &stream_reference_velocity_ ) == HEALTH_OK;
	bool stream_sp_pos_ok = stream_check( &stream_setpoint_position_ ) == HEALTH_OK;
	bool stream_sp_vel_ok = stream_check( &stream_setpoint_velocity_ ) == HEALTH_OK;
	bool stream_sp_accel_ok = stream_check( &stream_setpoint_acceleration_ ) == HEALTH_OK;

	geometry_msgs::PoseStamped goal_pos;
	geometry_msgs::TwistStamped goal_vel;
	geometry_msgs::AccelStamped goal_accel;
	mavros_msgs::AttitudeTarget goal_att;

	goal_att.orientation.w = 1.0f;	//Just to make sure the quaternion is initialized correctly

	double dt = (timerCallback.current_real - timerCallback.last_real).toSec();

	if( stream_sp_accel_ok ) {
		goal_accel = stream_setpoint_velocity_.data;
		do_control_accel = true;
	} else if( stream_sp_vel_ok && stream_ref_vel_ok ) {
		goal_vel = stream_setpoint_velocity_.data;

		do_control_vel = true;
		do_control_accel = true;
	} else if ( stream_sp_pos_ok && stream_ref_pos_ok && stream_ref_vel_ok ){
		goal_pos = stream_setpoint_position_.data;

		do_control_pos = true;
		do_control_vel = true;
		do_control_accel = true;
	}

	//If we have a setpoint stream that is OK, and a
	if( do_control_pos || do_control_vel || do_control_accel ) {
		//Position Control
		if( do_control_pos ) {
			goal_vel.header.frame_id = param_control_frame_id_;
			goal_vel.header.stamp = timerCallback.current_real;

			goal_vel.twist.linear.x = controller_pos_x.step( dt,
															 goal_pos.pose.position.x,
															 stream_reference_position_.data.pose.position.x,
															 stream_reference_velocity_.data.twist.linear.x );
			goal_vel.twist.linear.y = controller_pos_y.step( dt,
															 goal_pos.pose.position.y,
															 stream_reference_position_.data.pose.position.y,
															 stream_reference_velocity_.data.twist.linear.y );
			goal_vel.twist.linear.z = controller_pos_z.step( dt,
															 goal_pos.pose.position.z,
															 stream_reference_position_.data.pose.position.z,
															 stream_reference_velocity_.data.twist.linear.z );
		} else {
			//Prevent PID wind-up
			controller_pos_x.reset( stream_reference_position_.data.pose.position.x );
			controller_pos_y.reset( stream_reference_position_.data.pose.position.y );
			controller_pos_z.reset( stream_reference_position_.data.pose.position.z );
		}

		//Velocity Controller
		if( do_control_vel ) {
			goal_accel.header.frame_id = stream_setpoint_velocity_.data.header.frame_id;
			goal_accel.header.stamp = timerCallback.current_real;

			goal_accel.twist.linear.x = controller_vel_x.step( dt,
															   goal_vel.twist.linear.x,
															   stream_reference_velocity_.data.twist.linear.x );
			goal_accel.twist.linear.y = controller_vel_y.step( dt,
															   goal_vel.twist.linear.y,
															   stream_reference_velocity_.data.twist.linear.y );
			goal_accel.twist.linear.z = controller_vel_z.step( dt,
															   goal_vel.twist.linear.z,
															   stream_reference_velocity_.data.twist.linear.z );
		} else {
			//Prevent PID wind-up
			controller_vel_x.reset( stream_reference_position_.data.twist.linear.x );
			controller_vel_y.reset( stream_reference_position_.data.twist.linear.y );
			controller_vel_z.reset( stream_reference_position_.data.twist.linear.z );
		}

		if( do_control_accel ) {
			//Calculate goal accelerations
			double Tx = vel_x_pid.step(goalVelocity.twist.linear.x, bodyVelocity.linear.x );
			double Ty = vel_y_pid.step(goalVelocity.twist.linear.y, bodyVelocity.linear.y );
			double Tz = vel_z_pid.step(goalVelocity.twist.linear.z, bodyVelocity.linear.z ) + param_throttle_mid_; //TODO: Add in a hover approximation

			//Thrust Calculation
			tf2::Vector3 gThrust(Tx, Ty, Tz);
			tf2::Vector3 xyThrust(Tx, Ty, 0.0);

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
			if( gThrust.length() > param_throttle_max ) {
				//If thrust_z larger than thr_max, limit throttle, set xy to 0
				if( gThrust.getZ() > param_throttle_max ) {
					gThrust.setX( 0.0 );
					gThrust.setY( 0.0 );
					gThrust.setZ( param_throttle_max_ );
					ROS_WARN_THROTTLE(0.1, "Too much thrust, scaling Z");
				} else { //The XY thrust is the cause of the over-thrust, so scale down
					double thrust_xy_max = sqrtf( ( param_throttle_max_ * param_throttle_max_ ) - ( gThrust.getZ() * gThrust.getZ() ) );
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
			} else {
				//No thrust commanded, align with straight up
				body_z.setZ( 1.0f );
				ROS_INFO("No thrust command, keeping zero Z orientation!");
			}

			double yaw_c = 0.0f;

			if( do_control_pos ) {
				double roll_c, pitch_c;
				tf2::Quaternion q_c( goal_pos.pose.orientation.x,
									 goal_pos.pose.orientation.x,
									 goal_pos.pose.orientation.x,
									 goal_pos.pose.orientation.w );

				tf2::Matrix3x3( q_c ).getRPY( roll_c, pitch_c, yaw_c );
			}

			//Check to make sure the thrust vector is in the Z axis, or on the XY plane
			if (fabs(body_z.getZ()) > SIGMA) {
				//Get a vector that aligns the Y axis with pi/2
				tf::Vector3 yaw_r( sin( yaw_c ), cos( yaw_c ), 0.0f );
				//Get the orthagonal vector to that (which will have correct pitch)
				body_x = yaw_r.cross( body_z );

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
			tf2::Matrix3x3 R( body_x.getX(),
							  body_y.getX(),
							  body_z.getX(),
							  body_x.getY(),
							  body_y.getY(),
							  body_z.getY(),
							  body_x.getZ(),
							  body_y.getZ(),
							  body_z.getZ() );

			//Get quaternion from R
			R.getRotation( goalThrustRotation );
			goalThrust = gThrust.length();

			tf::quaternionTFToMsg( goalThrustRotation, outputAttitude.orientation );
			outputAttitude.thrust = goalThrust;
			outputAttitude.body_rate.z = goalYawRate;

		} else {
			//Prevent body z integrator wind-up
			integrator_body_rate_z_ = 0.0f;
		}

		//Do orientation control
		if( do_control_pos ) {
			goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE | goal_att.IGNORE_YAW_RATE;
		} else if( do_control_vel ) {
			goal_att.body_rate.z = stream_setpoint_velocity_.data.accel.angular.z;
			goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE;
		} else if ( do_control_accel ) {
			integrator_body_rate_z_ += dt * stream_setpoint_acceleration_.data.accel.angular.z;
			goal_att.body_rate.z = integrator_body_rate_z_;
			goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE;
		}

		goal_att.orientation = stream_setpoint_position_.data.pose.orientation;


	} else {
		ROS_ERROR("Timeout detected! Failsafe attitude output!");

		//Prevent PID wind-up
		controller_pos_x.reset( stream_reference_position_.data.pose.position.x );
		controller_pos_y.reset( stream_reference_position_.data.pose.position.y );
		controller_pos_z.reset( stream_reference_position_.data.pose.position.z );
		controller_vel_x.reset( stream_reference_position_.data.twist.linear.x );
		controller_vel_y.reset( stream_reference_position_.data.twist.linear.y );
		controller_vel_z.reset( stream_reference_position_.data.twist.linear.z );
		integrator_body_rate_z_ = 0.0f;

		goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE | goal_att.IGNORE_YAW_RATE | goal_att.IGNORE_ATTITUDE;
		goal_att.thrust = 0.0f;
	}

	goal_att.header.frame_id = param_control_frame_id_;
	goal_att.header.stamp = timerCallback.current_real;

	pub_output_attitude_.publish( goal_att );

	if( do_control_accel )
		pub_output_acceleration_.publish( goal_accel );

	if( do_control_vel )
		pub_output_velocity_.publish( goal_vel );

	if( do_control_pos )
		pub_output_position_.publish( goal_pos );
}

void Mavel::param_pid_init( mavel_params_pid* pid, std::string controller_name ) {
	nh_.param( controller_name + "/kp", pid->kp, pid->kp);
	nh_.param( controller_name + "/ki", pid->ki, pid->ki);
	nh_.param( controller_name + "/kd", pid->kd, pid->kd);
	nh_.param( controller_name + "/tau", pid->tau, pid->tau);
	nh_.param( controller_name + "/min", pid->min, pid->min);
	nh_.param( controller_name + "/max", pid->max, pid->max);
}

template<typename streamDataT>
void Mavel::stream_init( mavel_data_stream<streamDataT>* stream, double min_rate, std::string topic ) {
	stream->state = HEALTH_UNKNOWN;

	stream->count = 0;
	stream->stream_count = floor(2 * min_rate);
	stream->timeout = ros::Duration( 1.0f / min_rate );

	stream->data.header.stamp = ros::Time(0);
	stream->stream_topic = topic;
}

template<typename streamDataT>
void Mavel::stream_update( mavel_data_stream<streamDataT>* stream, <streamDataT>* data ) {
	stream->data = *data;
	stream->count++;
}

template<typename streamDataT>
mavel_data_stream_states Mavel::stream_check( mavel_data_stream<streamDataT>* stream, ros::Time time_now ) {
	//If the latest data in the stream is older than the timeout
	if( ( time_now - stream->data.header.stamp ) > stream->timeout ) {
		//If the stream was previously OK, then send warning
		if( stream->state == HEALTH_OK ) {
			stream->state = HEALTH_TIMEOUT;
			ROS_WARN("Control timeout on: %s", stream->stream_topic.c_str() );
		}

		//Reset the stream counter
		stream->count = 0;
	} else {	//We have fresh data
		//If we have established a stream, and it wasn't already OK
		if( ( stream->count > stream->stream_count ) && ( stream->state != HEALTH_OK ) {
			stream->state = HEALTH_OK;
			ROS_INFO("Control connected on: %s", stream->stream_topic.c_str() );
		}
	}

	return steam->state;
}

/*
	double param_loop_rate = 20.0;
	double param_trottle_threshold = 0.1;
	bool thr_threshold_passed = false;

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
