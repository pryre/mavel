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

Mavel::Mavel() :
	nh_(),
	nhp_( "~" ),
	control_started_(false),
	control_fatal_(false),
	param_got_valid_tri_(false),
	param_allow_controller_reset_(false),
	param_use_pct_control_(false),
	ref_path_(nhp_),
	dyncfg_settings_(ros::NodeHandle(nhp_)),
	controller_pos_x_(ros::NodeHandle(nhp_, "control/pos/x")),
	controller_pos_y_(ros::NodeHandle(nhp_, "control/pos/y")),
	controller_pos_z_(ros::NodeHandle(nhp_, "control/pos/z")),
	controller_vel_x_(ros::NodeHandle(nhp_, "control/vel/x")),
	controller_vel_y_(ros::NodeHandle(nhp_, "control/vel/y")),
	controller_vel_z_(ros::NodeHandle(nhp_, "control/vel/z")) {

	dyncfg_settings_.setCallback(boost::bind(&Mavel::callback_cfg_settings, this, _1, _2));

	// Load static parameters
	nhp_.param( "control_rate", param_rate_control_, 50.0 );
	nhp_.param( "control_frame", param_control_frame_id_, std::string("map") );
	ref_path_.set_frame_id(param_control_frame_id_);

	//Data streams
	nhp_.param( "min_rate/state/odometry", param_stream_min_rate_state_odometry_, 20.0 );
	nhp_.param( "min_rate/state/mav_state", param_stream_min_rate_state_mav_, 0.2 );
	nhp_.param( "min_rate/reference/triplet", param_stream_min_rate_reference_triplet_, 20.0 );

	/*
	//nhp_.param( "uav_mass", param_uav_mass_, 0.0 );
	//ROS_ASSERT_MSG(param_uav_mass_ > 0.0, "Error: uav_mass is invalid (<0.0)!");

	nhp_.param( "tilt_max", param_tilt_max_, 0.39 );

	nhp_.param( "throttle/min", param_throttle_min_, 0.0 );
	nhp_.param( "throttle/mid", param_throttle_mid_, 0.5 );
	nhp_.param( "throttle/max", param_throttle_max_, 0.9 );

	nhp_.param( "failsafe_land_vel", param_land_vel_, -0.2 );
	nhp_.param( "failsafe_output_on_fatal", param_output_low_on_fatal_, false );
	nhp_.param( "allow_timeout_position", param_allow_timeout_position_, false );
	nhp_.param( "allow_controller_reset", param_allow_controller_reset_, false );
	*/

	//Sanity check some parameters
	if( ( param_land_vel_ >= 0.0) ) {
		 ROS_FATAL("Invalid parameter settings, quiting mavel");
		 ros::shutdown();
	}

	stream_state_odometry_ = stream_init<nav_msgs::Odometry>( param_stream_min_rate_state_odometry_, "state/odometry" );
	stream_state_mav_ = stream_init<mavros_msgs::State>( param_stream_min_rate_state_mav_, "state/mav_state" );
	stream_reference_triplet_ = stream_init<mavros_msgs::PositionTarget>( param_stream_min_rate_reference_triplet_, "reference/triplet" );

	//Publishers
	pub_output_attitude_ = nhp_.advertise<mavros_msgs::AttitudeTarget>( "command/attitude", 100 );
	//pub_output_wrench_ = nhp_.advertise<geometry_msgs::WrenchStamped>( "feedback/wrench", 100 ); 	//XXX: Currently unused
	pub_output_acceleration_ = nhp_.advertise<geometry_msgs::AccelStamped>( "feedback/accel", 100 );
	pub_output_velocity_ = nhp_.advertise<geometry_msgs::TwistStamped>( "feedback/twist", 100 );
	pub_output_position_ = nhp_.advertise<geometry_msgs::PoseStamped>( "feedback/pose", 100 );
	pub_output_triplet_ = nhp_.advertise<mavros_msgs::PositionTarget>( "feedback/triplet", 100 );

	//Subscribers
	sub_state_odometry_ = nhp_.subscribe<nav_msgs::Odometry>( "state/odometry", 10, &Mavel::state_odometry_cb, this );
	sub_state_mav_ = nhp_.subscribe<mavros_msgs::State>( "state/mav_state", 10, &Mavel::state_mav_cb, this );
	sub_reference_triplet_ = nhp_.subscribe<mavros_msgs::PositionTarget>( "reference/triplet", 10, &Mavel::reference_triplet_cb, this );

	//Wait for streams before starting
	ROS_INFO("Mavel ready, waiting for control inputs...");

	//Timers for controllers
	timer_controller_ = nhp_.createTimer( ros::Duration( 1.0 / param_rate_control_ ), &Mavel::controller_cb, this );
}

Mavel::~Mavel() {
}

void Mavel::callback_cfg_settings( mavel::MavelParamsConfig &config, uint32_t level ) {
	param_allow_timeout_position_ = config.allow_timeout_position;
	param_allow_controller_reset_ = config.allow_controller_reset;
	param_use_pct_control_ = config.use_pct_control;

	param_tilt_max_ = config.tilt_max;

	param_throttle_min_ = config.throttle_min;
	param_throttle_mid_ = config.throttle_mid;
	param_throttle_max_ = config.throttle_max;

	param_output_low_on_fatal_ = config.failsafe_output_on_fatal;
	param_land_vel_ = config.failsafe_land_vel;

	//param_uav_mass_ = ...
}

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

void Mavel::controller_cb( const ros::TimerEvent& te ) {
	bool stream_state_odom_ok = stream_check( stream_state_odometry_, te.current_real ) == HEALTH_OK;
	bool stream_ref_tri_ok = stream_check( stream_reference_triplet_, te.current_real ) == HEALTH_OK;
	bool stream_ref_path_ok = ref_path_.has_reference(te.current_real); //Don't use a real stream, as it's more of a once off

	bool reference_ok = ( stream_state_odom_ok && ( stream_ref_path_ok ||
													stream_ref_tri_ok ) );

	bool state_ok = stream_state_odom_ok;
	bool arm_ok = flight_ready(te.current_real);

	mavros_msgs::AttitudeTarget msg_out;
	msg_out.orientation.w = 1.0;	//Just to make sure the quaternion is initialized correctly

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
		if( !arm_ok || !state_ok )
			control_fatal_ = true;

		if( control_fatal_ ) {
			if(!state_ok) {
				ROS_ERROR_THROTTLE( 2.0, "[PANIC] Reference error! Failsafe enabled!");
			} else if( !arm_ok ) {
				ROS_ERROR_THROTTLE( 2.0, "[PANIC] Arming error! Failsafe enabled!");
				if(param_allow_controller_reset_) {
					ROS_WARN("Mavel resetting controller");

					//Reset control inputs
					ref_path_.clear_reference();
					stream_reference_triplet_.data.header.stamp = ros::Time(0);

					//Reset control state
					control_started_ = false;
					control_fatal_ = false;
				}
			} else {
				ROS_FATAL("[PANIC] MAVEL ENTERED FAILSAFE UNKNOWN STATE!");
			}

			do_failsafe( te, msg_out );
		} else {
			do_control( te, msg_out );
		}
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

	//Real input streams
	bool stream_ref_tri_ok = stream_check( stream_reference_triplet_, te.current_real ) == HEALTH_OK;

	geometry_msgs::Point l_pos_ref;
	l_pos_ref.x = 0.0;
	l_pos_ref.y = 0.0;
	l_pos_ref.z = 0.0;

	geometry_msgs::Vector3 l_vel_ref;
	l_vel_ref.x = 0.0;
	l_vel_ref.y = 0.0;
	l_vel_ref.z = 0.0;

	double rot_ref = 0.0;
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
			rot_ref = goal_tri.yaw;

			l_vel_ref = goal_tri.velocity;
			r_rot_ref = goal_tri.yaw_rate;

			l_acc_ref = goal_tri.acceleration_or_force;

			do_control_pos = true;
			do_control_vel = true;
			do_control_accel = true;
		} else if(goal_tri.type_mask == TRIPLET_TRAJ_PVEL) {
			l_pos_ref = goal_tri.position;
			rot_ref = goal_tri.yaw;

			l_vel_ref = goal_tri.velocity;
			r_rot_ref = goal_tri.yaw_rate;

			do_control_pos = true;
			do_control_vel = true;
			do_control_accel = true;
		} else if(goal_tri.type_mask == TRIPLET_SETP_POS) {
			l_pos_ref = goal_tri.position;
			rot_ref = goal_tri.yaw;

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

	if(param_use_pct_control_) {
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
			l_acc_ref.x += controller_vel_x_.step( dt, l_vel_ref.x, state_vel.getX() );
			l_acc_ref.y += controller_vel_y_.step( dt, l_vel_ref.y, state_vel.getY() );
			l_acc_ref.z += controller_vel_z_.step( dt, l_vel_ref.z, state_vel.getZ() );
		} else {
			//Prevent PID wind-up
			controller_vel_x_.reset( state_vel.getX() );
			controller_vel_y_.reset( state_vel.getY() );
			controller_vel_z_.reset( state_vel.getZ() );
		}
	} else {
		geometry_msgs::Point eff_pos;
		eff_pos.x = 0.0;
		eff_pos.y = 0.0;
		eff_pos.z = 0.0;

		geometry_msgs::Vector3 eff_vel;
		eff_vel.x = 0.0;
		eff_vel.y = 0.0;
		eff_vel.z = 0.0;

		//Position/Trajectory/Path Control
		if( do_control_pos ) {
			eff_pos.x = controller_pos_x_.step( dt, l_pos_ref.x, state_tf.getOrigin().getX() );
			eff_pos.y = controller_pos_y_.step( dt, l_pos_ref.y, state_tf.getOrigin().getY() );
			eff_pos.z = controller_pos_z_.step( dt, l_pos_ref.z, state_tf.getOrigin().getZ() );
		} else {
			//Prevent PID wind-up
			controller_pos_x_.reset( state_tf.getOrigin().getX() );
			controller_pos_y_.reset( state_tf.getOrigin().getY() );
			controller_pos_z_.reset( state_tf.getOrigin().getZ() );
		}

		//Velocity Controller
		if( do_control_vel ) {
			eff_vel.x = controller_vel_x_.step( dt, l_vel_ref.x, state_vel.getX() );
			eff_vel.y = controller_vel_y_.step( dt, l_vel_ref.y, state_vel.getY() );
			eff_vel.z = controller_vel_z_.step( dt, l_vel_ref.z, state_vel.getZ() );
		} else {
			//Prevent PID wind-up
			controller_vel_x_.reset( state_vel.getX() );
			controller_vel_y_.reset( state_vel.getY() );
			controller_vel_z_.reset( state_vel.getZ() );
		}

		l_acc_ref.x += eff_pos.x + eff_vel.x;
		l_acc_ref.y += eff_pos.y + eff_vel.y;
		l_acc_ref.z += eff_pos.z + eff_vel.z;
	}

	tf2::Vector3 a(l_acc_ref.x, l_acc_ref.y, l_acc_ref.z + GRAVITY);
	tf2::Vector3 T(0.0,0.0,0.0);

	if( do_control_accel ) {
		//XXX:	The use of param_uav_mass_ is a bit redundant, but it allows
		//		for a nicer use of the vel->accel->norm_thrust chain, and
		//		provides us with better feedback throughout the rest of the
		//		control process.
		/*
		double est_force_max = (param_uav_mass_ * GRAVITY ) / param_throttle_mid_;
		T = (param_uav_mass_ * a);

		tf2::Vector3 nThrust = T / est_force_max;
		*/
		//Thrust Calculation
		tf2::Vector3 nThrust = param_throttle_mid_ * ( a /  GRAVITY );
		tf2::Vector3 xyThrust(nThrust.getX(), nThrust.getY(), 0.0);
		tf2::Quaternion goalThrustRotation(0.0, 0.0, 0.0, 1.0);

		//If xy thrust vector length greater than 0.01
		if( xyThrust.length() > MIN_DIST ) {
			//Limit horizontal thrust by z thrust
			double thrust_xy_max = nThrust.getZ() * std::tan( param_tilt_max_ );

			//If thrust_sp_xy_len > thrust_xy_max
			if( xyThrust.length() > thrust_xy_max ) {
				//Scale the XY thrust setpoint down
				double k = thrust_xy_max / xyThrust.length();
				nThrust.setX( k*nThrust.getX() );
				nThrust.setY( k*nThrust.getY() );
				xyThrust.setX( nThrust.getX() );
				xyThrust.setY( nThrust.getY() );
			}
		}

		//If thrust_abs > thr_max
		if( nThrust.length() > param_throttle_max_ ) {
			//If thrust_z larger than thr_max, limit throttle, set xy to 0
			if( nThrust.getZ() > param_throttle_max_ ) {
				nThrust.setX( 0.0 );
				nThrust.setY( 0.0 );
				nThrust.setZ( param_throttle_max_ );
				ROS_WARN_THROTTLE( 1.0, "Too much thrust, scaling Z" );
			} else { //The XY thrust is the cause of the over-thrust, so scale down
				double thrust_xy_max = sqrtf( ( param_throttle_max_ * param_throttle_max_ ) - ( nThrust.getZ() * nThrust.getZ() ) );
				double k = thrust_xy_max / xyThrust.length();
				nThrust.setX( k * nThrust.getX() );
				nThrust.setY( k * nThrust.getY() );

				ROS_WARN_THROTTLE( 1.0, "Too much thrust, scaling XY" );
			}
		}

		//Create body_x, body_y, body_z
		tf2::Vector3 thrust_x;
		tf2::Vector3 thrust_y;
		tf2::Vector3 thrust_z;

		//thrust_abs > SIGMA
		if( nThrust.length() > SIGMA ) {
			//Get the direction of the thrust vector (and rotate to the body frame)
			thrust_z = nThrust.normalized();
		} else {
			//No thrust commanded, align with straight up
			thrust_z.setZ( 1.0 );
			//ROS_INFO("No thrust command, keeping zero Z orientation!");
		}

		//Check to make sure the thrust vector is in the Z axis, or on the XY plane
		if ( fabs( thrust_z.getZ() ) > SIGMA ) {
			//Get a vector that aligns the Y axis with goal yaw
			tf2::Vector3 yaw_r( -sin( rot_ref ), cos( rot_ref ), 0.0 );
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

		goal_att.orientation.w = goalThrustRotation.w();
		goal_att.orientation.x = goalThrustRotation.x();
		goal_att.orientation.y = goalThrustRotation.y();
		goal_att.orientation.z = goalThrustRotation.z();

		goal_att.thrust = nThrust.length();
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

		//Make sure to add in the rate reference if we have it
		if( (goal_tri.type_mask == TRIPLET_TRAJ_FULL) || (goal_tri.type_mask == TRIPLET_TRAJ_PVEL) )
			goal_att.body_rate.z = goal_tri.yaw_rate;

		//Still need to flag that we don't want to override anything
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
		/* XXX: Could be used if UAV mass is known
		geometry_msgs::WrenchStamped goal_wrench;

		goal_wrench.header = goal_tri.header;
		goal_wrench.header.stamp = te.current_real;
		goal_wrench.wrench.force.x = T.getX();
		goal_wrench.wrench.force.y = T.getY();
		goal_wrench.wrench.force.z = T.getZ();

		pub_output_wrench_.publish( goal_wrench );
		*/

		pub_output_triplet_.publish( goal_tri );
	}

	if( do_control_accel ) {
		geometry_msgs::AccelStamped goal_acc;

		goal_acc.header = goal_tri.header;
		goal_acc.header.stamp = te.current_real;
		goal_acc.accel.linear = l_acc_ref;
		goal_acc.accel.angular.x = 0.0;
		goal_acc.accel.angular.y = 0.0;
		goal_acc.accel.angular.z = 0.0;

		pub_output_acceleration_.publish( goal_acc );
	}

	if( do_control_vel ) {
		geometry_msgs::TwistStamped goal_vel;

		goal_vel.header = goal_tri.header;
		goal_vel.header.stamp = te.current_real;
		goal_vel.twist.linear = l_vel_ref;
		goal_vel.twist.angular.x = 0.0;
		goal_vel.twist.angular.y = 0.0;
		goal_vel.twist.angular.z = r_rot_ref;

		pub_output_velocity_.publish( goal_vel );
	}

	if( do_control_pos ) {
		geometry_msgs::PoseStamped goal_pos;

		goal_pos.header = goal_tri.header;
		goal_pos.header.stamp = te.current_real;
		goal_pos.pose.position = l_pos_ref;
		goal_pos.pose.orientation = goal_att.orientation;

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

