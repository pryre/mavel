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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <string>
#include <math.h>

#define SIGMA			0.000001f
#define MIN_DIST		0.01f

Mavel::Mavel() :
	nh_( "~" ),
	tfln_( tfbuffer_ ),
	integrator_body_rate_z_( 0.0 ) {

	nh_.param( "control_rate", param_rate_control_, 20.0 );

	nh_.param( "tilt_max", param_tilt_max_, 0.39 );

	nh_.param( "throttle/min", param_throttle_min_, 0.0 );
	nh_.param( "throttle/mid", param_throttle_mid_, 0.5 );
	nh_.param( "throttle/max", param_throttle_max_, 0.9 );

	nh_.param( "control_frame", param_control_frame_id_, std::string("world") );

	//PID Controllers
	param_pid_init( &param_pid_pos_x, "pid/position/x" );
	param_pid_init( &param_pid_pos_y, "pid/position/y" );
	param_pid_init( &param_pid_pos_z, "pid/position/z" );
	param_pid_init( &param_pid_vel_x, "pid/velocity/x" );
	param_pid_init( &param_pid_vel_y, "pid/velocity/y" );
	param_pid_init( &param_pid_vel_z, "pid/velocity/z" );

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
	nh_.param( "stream/min_rate/reference/odometry", param_stream_min_rate_reference_odometry_, 25.0 );
	nh_.param( "stream/min_rate/setpoint/position", param_stream_min_rate_setpoint_position_, 5.0 );
	nh_.param( "stream/min_rate/setpoint/velocity", param_stream_min_rate_setpoint_velocity_, 25.0 );
	nh_.param( "stream/min_rate/setpoint/acceleration", param_stream_min_rate_setpoint_acceleration_, 50.0 );

	nh_.param( "stream/topic/reference/odometry", topic_input_odometry_reference_, std::string("reference/odometry") );

	nh_.param( "stream/topic/setpoint/position", topic_input_position_setpoint_, std::string("setpoint/pose") );
	nh_.param( "stream/topic/setpoint/velocity", topic_input_velocity_setpoint_, std::string("setpoint/cmd_vel") );
	nh_.param( "stream/topic/setpoint/acceleration", topic_input_acceleration_setpoint_, std::string("setpoint/accel") );

	nh_.param( "stream/topic/feedback/position", topic_output_position_, std::string("control/pose") );
	nh_.param( "stream/topic/feedback/velocity", topic_output_velocity_, std::string("control/cmd_vel") );
	nh_.param( "stream/topic/feedback/acceleration", topic_output_acceleration_, std::string("control/accel") );
	nh_.param( "stream/topic/feedback/attitude", topic_output_attitude_, std::string("control/attitude") );

	stream_init( &stream_reference_odometry_, param_stream_min_rate_reference_odometry_, topic_input_odometry_reference_ );
	stream_init( &stream_setpoint_position_, param_stream_min_rate_setpoint_position_, topic_input_position_setpoint_ );
	stream_init( &stream_setpoint_velocity_, param_stream_min_rate_setpoint_velocity_, topic_input_velocity_setpoint_ );
	stream_init( &stream_setpoint_acceleration_, param_stream_min_rate_setpoint_acceleration_, topic_input_acceleration_setpoint_ );

	//Publishers
	pub_output_attitude_ = nh_.advertise<mavros_msgs::AttitudeTarget>( topic_output_attitude_, 100 );
	pub_output_acceleration_ = nh_.advertise<geometry_msgs::AccelStamped>( topic_output_acceleration_, 100 );
	pub_output_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>( topic_output_velocity_, 100 );
	pub_output_position_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_output_position_, 100 );

	//Subscribers
	sub_reference_odometry_ = nh_.subscribe<nav_msgs::Odometry>( topic_input_odometry_reference_, 100, &Mavel::reference_odometry_cb, this );
	sub_setpoint_acceleration_ = nh_.subscribe<geometry_msgs::AccelStamped>( topic_input_acceleration_setpoint_, 100, &Mavel::setpoint_acceleration_cb, this );
	sub_setpoint_velocity_ = nh_.subscribe<geometry_msgs::TwistStamped>( topic_input_velocity_setpoint_, 100, &Mavel::setpoint_velocity_cb, this );
	sub_setpoint_position_ = nh_.subscribe<geometry_msgs::PoseStamped>( topic_input_position_setpoint_, 100, &Mavel::setpoint_position_cb, this );

	/*
	bool wait_for_inputs = true;
	ros::Rate check_rate( param_rate_control_ );
	ROS_INFO("Mavel setup, waiting for control inputs...");

	while( ros::ok() && wait_for_inputs ) {
		ros::Time time_now = ros::Time::now();

		bool stream_ref_odom_ok = stream_check( &stream_reference_odometry_, time_now ) == HEALTH_OK;
		bool stream_sp_pos_ok = stream_check( &stream_setpoint_position_, time_now ) == HEALTH_OK;
		bool stream_sp_vel_ok = stream_check( &stream_setpoint_velocity_, time_now ) == HEALTH_OK;
		bool stream_sp_accel_ok = stream_check( &stream_setpoint_acceleration_, time_now ) == HEALTH_OK;

		if( ( stream_sp_accel_ok ) ||
			( stream_sp_vel_ok && stream_ref_odom_ok ) ||
			( stream_sp_pos_ok && stream_ref_odom_ok ) )
			wait_for_inputs = false;

		ros::spinOnce();
		check_rate.sleep();
	}
	*/

	//Timers for controllers
	timer_controller_ = nh_.createTimer( ros::Duration( 1.0 / param_rate_control_ ), &Mavel::controller_cb, this );

	ROS_INFO("Mavel running!");
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

void Mavel::reference_odometry_cb( const nav_msgs::Odometry msg_in ) {
	stream_update( &stream_reference_odometry_, &msg_in );
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

void Mavel::param_pid_init( mavel_params_pid* pid, std::string controller_name ) {
	nh_.param( controller_name + "/kp", pid->kp, 0.0);
	nh_.param( controller_name + "/ki", pid->ki, 0.0);
	nh_.param( controller_name + "/kd", pid->kd, 0.0);
	nh_.param( controller_name + "/tau", pid->tau, 0.0);
	nh_.param( controller_name + "/min", pid->min, -1.0);
	nh_.param( controller_name + "/max", pid->max, 1.0);
}

template<typename streamDataT>
void Mavel::stream_init( mavel_data_stream<streamDataT>* stream, const double min_rate, const std::string topic ) {
	stream->state = HEALTH_UNKNOWN;

	stream->count = 0;
	stream->stream_count = floor( 2 * min_rate );
	stream->timeout = ros::Duration( 1.0 / min_rate );

	stream->data.header.stamp = ros::Time( 0 );
	stream->stream_topic = topic;
}

template<typename streamDataT>
void Mavel::stream_update( mavel_data_stream<streamDataT>* stream, const streamDataT* data ) {
	stream->data = *data;
	stream->count++;
}

template<typename streamDataT>
mavel_data_stream_states Mavel::stream_check( mavel_data_stream<streamDataT>* stream, const ros::Time time_now ) {
	//If the latest data in the stream is older than the timeout
	if( ( time_now - stream->data.header.stamp ) > stream->timeout ) {
		//If the stream was previously OK, then send warning
		if( stream->state == HEALTH_OK ) {
			stream->state = HEALTH_TIMEOUT;
			ROS_WARN("Stream timeout on: %s", ( ros::this_node::getName() + "/" + stream->stream_topic ).c_str() );
		}

		//Reset the stream counter
		stream->count = 0;
	} else if( ( stream->count > stream->stream_count ) && ( stream->state != HEALTH_OK ) ) {
		//If we have established a stream, and it wasn't already OK
		stream->state = HEALTH_OK;
		ROS_INFO("Stream connected on: %s", ( ros::this_node::getName() + "/" + stream->stream_topic ).c_str() );
	}

	return stream->state;
}

void Mavel::controller_cb( const ros::TimerEvent& timerCallback ) {
	bool do_control_pos = false;
	bool do_control_vel = false;
	bool do_control_accel = false;

	geometry_msgs::PoseStamped goal_pos;
	geometry_msgs::TwistStamped goal_vel;
	geometry_msgs::AccelStamped goal_accel;
	mavros_msgs::AttitudeTarget goal_att;

	goal_att.orientation.w = 1.0;	//Just to make sure the quaternion is initialized correctly

	double dt = (timerCallback.current_real - timerCallback.last_real).toSec();

	bool stream_ref_odom_ok = stream_check( &stream_reference_odometry_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_sp_pos_ok = stream_check( &stream_setpoint_position_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_sp_vel_ok = stream_check( &stream_setpoint_velocity_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_sp_accel_ok = stream_check( &stream_setpoint_acceleration_, timerCallback.current_real ) == HEALTH_OK;

	if( stream_sp_accel_ok ) {
		goal_accel = stream_setpoint_acceleration_.data;
		do_control_accel = true;
	} else if( stream_sp_vel_ok && stream_ref_odom_ok ) {
		goal_vel = stream_setpoint_velocity_.data;

		do_control_vel = true;
		do_control_accel = true;
	} else if ( stream_sp_pos_ok && stream_ref_odom_ok ){
		goal_pos = stream_setpoint_position_.data;

		do_control_pos = true;
		do_control_vel = true;
		do_control_accel = true;
	} else if ( stream_ref_odom_ok ) {
		do_control_vel = true;
		do_control_accel = true;

		goal_vel.header.frame_id = param_control_frame_id_;
		goal_vel.header.stamp = timerCallback.current_real;
		goal_vel.twist.linear.z = -0.2;	//TODO: Parameter

		ROS_ERROR_THROTTLE( 2.0, "No setpoint data, failsafe landing mode!");
	}

	//If we have a setpoint stream that is OK, and a
	if( do_control_pos || do_control_vel || do_control_accel ) {
		//Position Control
		if( do_control_pos ) {
			goal_vel.header.frame_id = param_control_frame_id_;
			goal_vel.header.stamp = timerCallback.current_real;

			goal_vel.twist.linear.x = controller_pos_x.step( dt,
															 goal_pos.pose.position.x,
															 stream_reference_odometry_.data.pose.pose.position.x,
															 stream_reference_odometry_.data.twist.twist.linear.x );
			goal_vel.twist.linear.y = controller_pos_y.step( dt,
															 goal_pos.pose.position.y,
															 stream_reference_odometry_.data.pose.pose.position.y,
															 stream_reference_odometry_.data.twist.twist.linear.y );
			goal_vel.twist.linear.z = controller_pos_z.step( dt,
															 goal_pos.pose.position.z,
															 stream_reference_odometry_.data.pose.pose.position.z,
															 stream_reference_odometry_.data.twist.twist.linear.z );
		} else {
			//Prevent PID wind-up
			controller_pos_x.reset( stream_reference_odometry_.data.pose.pose.position.x );
			controller_pos_y.reset( stream_reference_odometry_.data.pose.pose.position.y );
			controller_pos_z.reset( stream_reference_odometry_.data.pose.pose.position.z );
		}

		//Velocity Controller
		if( do_control_vel ) {
			goal_accel.header.frame_id = stream_setpoint_velocity_.data.header.frame_id;
			goal_accel.header.stamp = timerCallback.current_real;

			goal_accel.accel.linear.x = controller_vel_x.step( dt,
															   goal_vel.twist.linear.x,
															   stream_reference_odometry_.data.twist.twist.linear.x );
			goal_accel.accel.linear.y = controller_vel_y.step( dt,
															   goal_vel.twist.linear.y,
															   stream_reference_odometry_.data.twist.twist.linear.y );
			goal_accel.accel.linear.z = controller_vel_z.step( dt,
															   goal_vel.twist.linear.z,
															   stream_reference_odometry_.data.twist.twist.linear.z );
		} else {
			//Prevent PID wind-up
			controller_vel_x.reset( stream_reference_odometry_.data.twist.twist.linear.x );
			controller_vel_y.reset( stream_reference_odometry_.data.twist.twist.linear.y );
			controller_vel_z.reset( stream_reference_odometry_.data.twist.twist.linear.z );
		}

		if( do_control_accel ) {
			//Calculate goal accelerations
			double Tx = goal_accel.accel.linear.x;
			double Ty = goal_accel.accel.linear.y;
			double Tz = goal_accel.accel.linear.z + param_throttle_mid_; //TODO: Add in a hover approximation

			//Thrust Calculation
			tf2::Vector3 gThrust(Tx, Ty, Tz);
			tf2::Vector3 xyThrust(Tx, Ty, 0.0);
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
			tf2::Vector3 body_x;
			tf2::Vector3 body_y;
			tf2::Vector3 body_z;

			//thrust_abs > SIGMA
			if( gThrust.length() > SIGMA) {
				//Get the direction of the thrust vector (and rotate to the body frame)
				body_z = gThrust.normalized();
			} else {
				//No thrust commanded, align with straight up
				body_z.setZ( 1.0 );
				//ROS_INFO("No thrust command, keeping zero Z orientation!");
			}

			double yaw_c = 0.0;

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
				tf2::Vector3 yaw_r( sin( yaw_c ), cos( yaw_c ), 0.0 );
				//Get the orthagonal vector to that (which will have correct pitch)
				body_x = yaw_r.cross( body_z );

				//keep nose to front while inverted upside down
				if (body_z.getZ() < 0.0)
					body_x = -body_x;

				body_x.normalize();
			} else {
				// desired thrust is in XY plane, set X downside to construct correct matrix,
				// but yaw component will not be used actually
				body_x.setZ( 1.0 );
				ROS_INFO( "No thrust command, keeping zero XY orientation!" );
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

			goal_att.orientation.w = goalThrustRotation.getW();
			goal_att.orientation.x = goalThrustRotation.getX();
			goal_att.orientation.y = goalThrustRotation.getY();
			goal_att.orientation.z = goalThrustRotation.getZ();

			goal_att.thrust = gThrust.length();
		} else {
			//Prevent body z integrator wind-up
			integrator_body_rate_z_ = 0.0;
		}

		//Do orientation control
		if( do_control_pos ) {
			goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE | goal_att.IGNORE_YAW_RATE;
		} else if( do_control_vel ) {
			goal_att.body_rate.z = stream_setpoint_velocity_.data.twist.angular.z;
			goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE;
		} else if ( do_control_accel ) {
			integrator_body_rate_z_ += dt * stream_setpoint_acceleration_.data.accel.angular.z;
			goal_att.body_rate.z = integrator_body_rate_z_;
			goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE;
		}
	} else {
		ROS_ERROR_THROTTLE( 2.0, "Timeout detected! Failsafe attitude output!");

		//Prevent PID wind-up
		controller_pos_x.reset( stream_reference_odometry_.data.pose.pose.position.x );
		controller_pos_y.reset( stream_reference_odometry_.data.pose.pose.position.y );
		controller_pos_z.reset( stream_reference_odometry_.data.pose.pose.position.z );
		controller_vel_x.reset( stream_reference_odometry_.data.twist.twist.linear.x );
		controller_vel_y.reset( stream_reference_odometry_.data.twist.twist.linear.y );
		controller_vel_z.reset( stream_reference_odometry_.data.twist.twist.linear.z );
		integrator_body_rate_z_ = 0.0;

		goal_att.type_mask |= goal_att.IGNORE_ROLL_RATE | goal_att.IGNORE_PITCH_RATE | goal_att.IGNORE_YAW_RATE | goal_att.IGNORE_ATTITUDE;
		goal_att.thrust = 0.0;
	}

	//Minimum throttle override
	if( goal_att.thrust < param_throttle_min_ )
		goal_att.thrust = param_throttle_min_;

	//Add in headers for attitude taget
	goal_att.header.frame_id = param_control_frame_id_;
	goal_att.header.stamp = timerCallback.current_real;

	pub_output_attitude_.publish( goal_att );

	//Handle the control feedback
	if( do_control_accel )
		pub_output_acceleration_.publish( goal_accel );

	if( do_control_vel )
		pub_output_velocity_.publish( goal_vel );

	if( do_control_pos )
		pub_output_position_.publish( goal_pos );
}

