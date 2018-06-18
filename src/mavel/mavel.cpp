#include <mavel/mavel.h>
#include <pid_controller_lib/pidController.h>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>

#include <tf2/LinearMath/Transform.h>
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
	nh_(),
	nhp_( "~" ),
	control_started_(false),
	control_fatal_(false),
	integrator_body_rate_z_( 0.0 ),
	controller_pos_x_(&nhp_, "control/pos/x"),
	controller_pos_y_(&nhp_, "control/pos/y"),
	controller_pos_z_(&nhp_, "control/pos/z"),
	controller_vel_x_(&nhp_, "control/vel/x"),
	controller_vel_y_(&nhp_, "control/vel/y"),
	controller_vel_z_(&nhp_, "control/vel/z") {

	nhp_.param( "control_rate", param_rate_control_, 20.0 );

	nhp_.param( "tilt_max", param_tilt_max_, 0.39 );

	nhp_.param( "throttle/min", param_throttle_min_, 0.0 );
	nhp_.param( "throttle/mid", param_throttle_mid_, 0.5 );
	nhp_.param( "throttle/max", param_throttle_max_, 0.9 );

	nhp_.param( "failsafe_land_vel", param_land_vel_, -0.2 );

	nhp_.param( "control_frame", param_control_frame_id_, std::string("world") );

	//Data streams
	nhp_.param( "stream/min_rate/reference/odometry", param_stream_min_rate_reference_odometry_, 25.0 );
	nhp_.param( "stream/min_rate/reference/state", param_stream_min_rate_reference_state_, 0.5 );
	nhp_.param( "stream/min_rate/setpoint/position", param_stream_min_rate_setpoint_position_, 5.0 );
	nhp_.param( "stream/min_rate/setpoint/velocity", param_stream_min_rate_setpoint_velocity_, 25.0 );
	nhp_.param( "stream/min_rate/setpoint/acceleration", param_stream_min_rate_setpoint_acceleration_, 50.0 );

	nhp_.param( "stream/topic/reference/odometry", topic_input_odometry_reference_, std::string("reference/odometry") );
	nhp_.param( "stream/topic/reference/state", topic_input_state_reference_, std::string("reference/state") );

	nhp_.param( "stream/topic/setpoint/position", topic_input_position_setpoint_, std::string("setpoint/pose") );
	nhp_.param( "stream/topic/setpoint/velocity", topic_input_velocity_setpoint_, std::string("setpoint/cmd_vel") );
	nhp_.param( "stream/topic/setpoint/acceleration", topic_input_acceleration_setpoint_, std::string("setpoint/accel") );

	nhp_.param( "stream/topic/feedback/position", topic_output_position_, std::string("control/pose") );
	nhp_.param( "stream/topic/feedback/velocity", topic_output_velocity_, std::string("control/cmd_vel") );
	nhp_.param( "stream/topic/feedback/acceleration", topic_output_acceleration_, std::string("control/accel") );
	nhp_.param( "stream/topic/feedback/attitude", topic_output_attitude_, std::string("control/attitude") );

	stream_init( &stream_reference_odometry_, param_stream_min_rate_reference_odometry_, topic_input_odometry_reference_ );
	stream_init( &stream_reference_state_, param_stream_min_rate_reference_state_, topic_input_state_reference_ );
	stream_init( &stream_setpoint_position_, param_stream_min_rate_setpoint_position_, topic_input_position_setpoint_ );
	stream_init( &stream_setpoint_velocity_, param_stream_min_rate_setpoint_velocity_, topic_input_velocity_setpoint_ );
	stream_init( &stream_setpoint_acceleration_, param_stream_min_rate_setpoint_acceleration_, topic_input_acceleration_setpoint_ );

	//Publishers
	pub_output_attitude_ = nhp_.advertise<mavros_msgs::AttitudeTarget>( topic_output_attitude_, 100 );
	pub_output_acceleration_ = nhp_.advertise<geometry_msgs::AccelStamped>( topic_output_acceleration_, 100 );
	pub_output_velocity_ = nhp_.advertise<geometry_msgs::TwistStamped>( topic_output_velocity_, 100 );
	pub_output_position_ = nhp_.advertise<geometry_msgs::PoseStamped>( topic_output_position_, 100 );

	//Subscribers
	sub_reference_odometry_ = nhp_.subscribe<nav_msgs::Odometry>( topic_input_odometry_reference_, 100, &Mavel::reference_odometry_cb, this );
	sub_reference_state_ = nhp_.subscribe<mavros_msgs::State>( topic_input_state_reference_, 100, &Mavel::reference_state_cb, this );
	sub_setpoint_acceleration_ = nhp_.subscribe<geometry_msgs::AccelStamped>( topic_input_acceleration_setpoint_, 100, &Mavel::setpoint_acceleration_cb, this );
	sub_setpoint_velocity_ = nhp_.subscribe<geometry_msgs::TwistStamped>( topic_input_velocity_setpoint_, 100, &Mavel::setpoint_velocity_cb, this );
	sub_setpoint_position_ = nhp_.subscribe<geometry_msgs::PoseStamped>( topic_input_position_setpoint_, 100, &Mavel::setpoint_position_cb, this );

	//Wait for streams before starting
	ROS_INFO("Mavel setup, waiting for control inputs...");

	//Timers for controllers
	timer_controller_ = nhp_.createTimer( ros::Duration( 1.0 / param_rate_control_ ), &Mavel::controller_cb, this );
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

void Mavel::reference_state_cb( const mavros_msgs::State msg_in ) {
	stream_update( &stream_reference_state_, &msg_in );
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
	bool stream_ref_odom_ok = stream_check( &stream_reference_odometry_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_ref_state_ok = stream_check( &stream_reference_state_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_sp_pos_ok = stream_check( &stream_setpoint_position_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_sp_vel_ok = stream_check( &stream_setpoint_velocity_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_sp_accel_ok = stream_check( &stream_setpoint_acceleration_, timerCallback.current_real ) == HEALTH_OK;

	bool setpoints_ok = ( stream_sp_accel_ok ) ||
						( stream_sp_vel_ok && stream_ref_odom_ok ) ||
						( stream_sp_pos_ok && stream_ref_odom_ok );

	bool reference_ok = stream_ref_odom_ok && stream_ref_state_ok;

	bool arm_ok = stream_reference_state_.data.armed;

	mavros_msgs::AttitudeTarget msg_out;
	msg_out.orientation.w = 1.0;	//Just to make sure the quaternion is initialized correctly

	if( !control_fatal_ ) {
		if( !control_started_ ) {
			if( reference_ok && setpoints_ok ) {
				ROS_INFO_THROTTLE( 2.0, "Mavel ready, waiting for mav to be armed");

				if( arm_ok ) {
					control_started_ = true;
					ROS_INFO("Mav armed starting position control!");
				}
			}

			do_failsafe( timerCallback, &msg_out );
		} else {
			if( arm_ok && reference_ok ) {
				do_control( timerCallback, &msg_out );
			} else {
				control_fatal_ = true;
			}
		}
	} else {
		ROS_ERROR_THROTTLE( 2.0, "[PANIC] Reference or arming error! Failsafe enabled!");
		do_failsafe( timerCallback, &msg_out );
	}


	//Add in headers for attitude taget
	msg_out.header.frame_id = param_control_frame_id_;
	msg_out.header.stamp = timerCallback.current_real;

	pub_output_attitude_.publish( msg_out );
}

void Mavel::do_control( const ros::TimerEvent& timerCallback, mavros_msgs::AttitudeTarget* goal_att ) {
	bool do_control_pos = false;
	bool do_control_vel = false;
	bool do_control_accel = false;

	geometry_msgs::PoseStamped goal_pos;
	geometry_msgs::TwistStamped goal_vel;
	geometry_msgs::AccelStamped goal_accel;

	tf2::Transform ref_tf;
	tf2::Vector3 ref_vel;

	double dt = (timerCallback.current_real - timerCallback.last_real).toSec();

	bool stream_sp_pos_ok = stream_check( &stream_setpoint_position_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_sp_vel_ok = stream_check( &stream_setpoint_velocity_, timerCallback.current_real ) == HEALTH_OK;
	bool stream_sp_accel_ok = stream_check( &stream_setpoint_acceleration_, timerCallback.current_real ) == HEALTH_OK;

	if( stream_sp_accel_ok ) {
		goal_accel = stream_setpoint_acceleration_.data;
		do_control_accel = true;
	} else if( stream_sp_vel_ok ) {
		goal_vel = stream_setpoint_velocity_.data;

		do_control_vel = true;
		do_control_accel = true;
	} else if ( stream_sp_pos_ok ){
		goal_pos = stream_setpoint_position_.data;

		do_control_pos = true;
		do_control_vel = true;
		do_control_accel = true;
	} else {
		goal_vel.header.frame_id = param_control_frame_id_;
		goal_vel.header.stamp = timerCallback.current_real;
		goal_vel.twist.linear.z = param_land_vel_;

		do_control_vel = true;
		do_control_accel = true;

		ROS_ERROR_THROTTLE( 2.0, "[Timeout] Setpoint error! Emergency landing!");
	}

	tf2::Vector3 tmp_pos( stream_reference_odometry_.data.pose.pose.position.x,
						  stream_reference_odometry_.data.pose.pose.position.y,
						  stream_reference_odometry_.data.pose.pose.position.z );

	tf2::Quaternion tmp_q( stream_reference_odometry_.data.pose.pose.orientation.x,
						   stream_reference_odometry_.data.pose.pose.orientation.y,
						   stream_reference_odometry_.data.pose.pose.orientation.z,
						   stream_reference_odometry_.data.pose.pose.orientation.w );

	ref_tf = tf2::Transform( tmp_q, tmp_pos );

	tf2::Quaternion rot_vel;
	rot_vel = ( tmp_q * tf2::Quaternion( stream_reference_odometry_.data.twist.twist.linear.x,
									   stream_reference_odometry_.data.twist.twist.linear.y,
									   stream_reference_odometry_.data.twist.twist.linear.z,
									   0.0 ) ) * tmp_q.inverse();

	ref_vel = tf2::Vector3( rot_vel.getX(), rot_vel.getY(), rot_vel.getZ() );

	//Position Control
	if( do_control_pos ) {
		goal_vel.header.frame_id = param_control_frame_id_;
		goal_vel.header.stamp = timerCallback.current_real;

		goal_vel.twist.linear.x = controller_pos_x_.step( dt, goal_pos.pose.position.x, ref_tf.getOrigin().getX(), ref_vel.getX() );
		goal_vel.twist.linear.y = controller_pos_y_.step( dt, goal_pos.pose.position.y, ref_tf.getOrigin().getY(), ref_vel.getY() );
		goal_vel.twist.linear.z = controller_pos_z_.step( dt, goal_pos.pose.position.z, ref_tf.getOrigin().getZ(), ref_vel.getZ() );
	} else {
		//Prevent PID wind-up
		controller_pos_x_.reset( ref_tf.getOrigin().getX() );
		controller_pos_y_.reset( ref_tf.getOrigin().getY() );
		controller_pos_z_.reset( ref_tf.getOrigin().getZ() );
	}

	//Velocity Controller
	if( do_control_vel ) {
		goal_accel.header.frame_id = stream_setpoint_velocity_.data.header.frame_id;
		goal_accel.header.stamp = timerCallback.current_real;

		goal_accel.accel.linear.x = controller_vel_x_.step( dt, goal_vel.twist.linear.x, ref_vel.getX() );
		goal_accel.accel.linear.y = controller_vel_y_.step( dt, goal_vel.twist.linear.y, ref_vel.getY() );
		goal_accel.accel.linear.z = controller_vel_z_.step( dt, goal_vel.twist.linear.z, ref_vel.getZ() );
	} else {
		//Prevent PID wind-up
		controller_vel_x_.reset( ref_vel.getX() );
		controller_vel_y_.reset( ref_vel.getY() );
		controller_vel_z_.reset( ref_vel.getZ() );
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
			tf2::Quaternion q_c( goal_pos.pose.orientation.x,
								 goal_pos.pose.orientation.y,
								 goal_pos.pose.orientation.z,
								 goal_pos.pose.orientation.w );

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

		goal_att->orientation.w = goalThrustRotation.getW();
		goal_att->orientation.x = goalThrustRotation.getX();
		goal_att->orientation.y = goalThrustRotation.getY();
		goal_att->orientation.z = goalThrustRotation.getZ();

		goal_att->thrust = gThrust.length();
	} else {
		//Prevent body z integrator wind-up
		integrator_body_rate_z_ = 0.0;
	}

	//Do orientation control
	if( do_control_pos ) {
		goal_att->type_mask |= goal_att->IGNORE_ROLL_RATE | goal_att->IGNORE_PITCH_RATE | goal_att->IGNORE_YAW_RATE;
	} else if( do_control_vel ) {
		goal_att->body_rate.z = stream_setpoint_velocity_.data.twist.angular.z;
		goal_att->type_mask |= goal_att->IGNORE_ROLL_RATE | goal_att->IGNORE_PITCH_RATE;
	} else if ( do_control_accel ) {
		integrator_body_rate_z_ += dt * stream_setpoint_acceleration_.data.accel.angular.z;
		goal_att->body_rate.z = integrator_body_rate_z_;
		goal_att->type_mask |= goal_att->IGNORE_ROLL_RATE | goal_att->IGNORE_PITCH_RATE;
	}

	//Minimum throttle override
	if( goal_att->thrust < param_throttle_min_ )
		goal_att->thrust = param_throttle_min_;

	//Handle the control feedback
	if( do_control_accel )
		pub_output_acceleration_.publish( goal_accel );

	if( do_control_vel )
		pub_output_velocity_.publish( goal_vel );

	if( do_control_pos )
		pub_output_position_.publish( goal_pos );
}

void Mavel::do_failsafe( const ros::TimerEvent& timerCallback, mavros_msgs::AttitudeTarget* goal_att ) {
	//Prevent PID wind-up
	controller_pos_x_.reset( 0.0 );
	controller_pos_y_.reset( 0.0 );
	controller_pos_z_.reset( 0.0 );
	controller_vel_x_.reset( 0.0 );
	controller_vel_y_.reset( 0.0 );
	controller_vel_z_.reset( 0.0 );
	integrator_body_rate_z_ = 0.0;

	goal_att->orientation.w = 1.0;
	goal_att->orientation.x = 0.0;
	goal_att->orientation.y = 0.0;
	goal_att->orientation.z = 0.0;
	goal_att->body_rate.x = 0.0;
	goal_att->body_rate.y = 0.0;
	goal_att->body_rate.z = 0.0;
	goal_att->type_mask |= goal_att->IGNORE_ROLL_RATE | goal_att->IGNORE_PITCH_RATE | goal_att->IGNORE_YAW_RATE | goal_att->IGNORE_ATTITUDE;
	goal_att->thrust = 0.0;
}

