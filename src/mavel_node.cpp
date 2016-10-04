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

geometry_msgs::PoseStamped currentPose;
geometry_msgs::TwistStamped goalVelocity;

bool DEBUG_MSGS;

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	currentPose = *msg;
}

void cmd_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	goalVelocity = *msg;
}
/*
geometry_msgs::Vector3 toEuler(geometry_msgs::Quaternion q) {
    geometry_msgs::Vector3 e;

    double q2sqr = q.y * q.y;
    double t0 = -2.0 * (q2sqr + q.z * q.z) + 1.0;
    double t1 = +2.0 * (q.x * q.y + q.w * q.z);
    double t2 = -2.0 * (q.x * q.z - q.w * q.y);
    double t3 = +2.0 * (q.y * q.z + q.w * q.x);
    double t4 = -2.0 * (q.x * q.x + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    e.x = atan2(t3, t4);
    e.y = asin(t2);
    e.z = atan2(t1, t0);

    return e;
}

geometry_msgs::Quaternion toQuaternion(geometry_msgs::Vector3 e) {
    geometry_msgs::Quaternion q;

    double t0 = cos(e.z * 0.5);
    double t1 = sin(e.z * 0.5);
    double t2 = cos(e.x * 0.5);
    double t3 = sin(e.x * 0.5);
    double t4 = cos(e.y * 0.5);
    double t5 = sin(e.y * 0.5);

    q.w = t2 * t4 * t0 + t3 * t5 * t1;
    q.x = t3 * t4 * t0 - t2 * t5 * t1;
    q.y = t2 * t5 * t0 + t3 * t4 * t1;
    q.z = t2 * t4 * t1 - t3 * t5 * t0;

	return q;
}
*/
//================================//
// Main Function                  //
//================================//
int main(int argc, char **argv) {
	//================================//
	// Initialize node                //
	//================================//
	ros::init(argc, argv, "mavel" );
	ros::NodeHandle nh( ros::this_node::getName() );

	DEBUG_MSGS = true;
	double loopRate = 20.0;
	double commandTimeout = 0.1; //TODO: Params
	bool freshCommands = false;		//The thread will be locked to only sending safety values
	bool freshTransform = false;	//The thread will be locked to only sending safety values

	//TODO: Dynamic params
	double param_yaw_rate_ff = 10.0;
	double param_throttle_min = 0.15;
	double param_throttle_mid = 0.5;
	double param_throttle_max = 0.85;
	double param_vel_xy_pid_p = 0.15;
	double param_vel_xy_pid_i = 0.01;
	double param_vel_xy_pid_d = 0.05;
	double param_vel_xy_pid_min = -1.0;
	double param_vel_xy_pid_max = 1.0;
	double param_vel_z_pid_p = 0.1;
	double param_vel_z_pid_i = 0.1;
	double param_vel_z_pid_d = 0.05;
	double param_vel_z_pid_min = -(param_throttle_mid - param_throttle_min);	//Set based on the throttle settings
	double param_vel_z_pid_max = param_throttle_max - param_throttle_mid;		//Otherwise they would just cause additional pid windup
	double param_tilt_max = 0.39;	//22.5 Deg
	double param_thr_max = 1.0;	//100%

	mavros_msgs::AttitudeTarget outputAttitude;
	//TODO: seperate rates are not yet properly supported, just ignore all
	outputAttitude.type_mask = outputAttitude.IGNORE_ROLL_RATE + outputAttitude.IGNORE_PITCH_RATE;	//We want to send yaw rate, but not other rates
	outputAttitude.header.frame_id = "/fcu/goal";

	pid vel_x_pid( 1.0/loopRate, param_vel_xy_pid_p, param_vel_xy_pid_i, param_vel_xy_pid_d, param_vel_xy_pid_min, param_vel_xy_pid_max );
	pid vel_y_pid( 1.0/loopRate, param_vel_xy_pid_p, param_vel_xy_pid_i, param_vel_xy_pid_d, param_vel_xy_pid_min, param_vel_xy_pid_max );
	pid vel_z_pid( 1.0/loopRate, param_vel_z_pid_p, param_vel_z_pid_i, param_vel_z_pid_d, param_vel_z_pid_min, param_vel_z_pid_max);

	//Subscribers
	ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("cmd_vel", 10, cmd_vel_cb);

	ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
		("/mavros/setpoint_raw/attitude", 10);

	//Initialize TF
	tf::TransformListener tfln;
	tf::TransformBroadcaster tfbr;

	ros::Rate rate(loopRate);
	ros::spinOnce();

	//Sit and wait until the inputs are ready
	while( !freshCommands || !freshTransform ) {
		freshCommands = commandTimeout > ( ros::Time::now() - goalVelocity.header.stamp ).toSec();
		freshTransform = tfln.waitForTransform( "/world", "/fcu", ros::Time::now(), ros::Duration(1.0) );

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
		double currentHeading = 0.0;

		tf::Quaternion goalThrustRotation;
		double goalThrust = param_throttle_min;

		//Check to make sure there have been new commands
		if( commandTimeout < ( ros::Time::now() - goalVelocity.header.stamp ).toSec() ) {
			freshCommands = false;
			ROS_ERROR_ONCE( "[Timeout] No new velocity commands" );
		}

		if( freshTransform ) {
			try {
				//Get the latest pose of the fcu in the world
				tfln.lookupTransform("/world", "/fcu", ros::Time(0), currentTransform);

				//Get the latest twist (velocity) estimate of the fcu relative to the world
				tfln.lookupTwist("fcu/ref", "world", "fcu/ref", tf::Point(), "world", ros::Time(0), ros::Duration(0.5), worldVelocity);
			}

			catch (tf::TransformException ex) {
				ROS_ERROR( "%s",ex.what() );
				freshTransform = false;
			}

			//Only bother doing these steps if there is actually a new transform, as they will cause nan errors
			if( freshTransform ) {

				//Prepare the output rotation goal message
				//Need to use the current rotation to get the yaw rate to work (that it doesn't try to yaw if no commands are sent)
				//geometry_msgs::Quaternion qRot;
				//quaternionTFToMsg( currentTransform.getRotation(), qRot );
				//geometry_msgs::Vector3 rot = toEuler( qRot );
				geometry_msgs::Vector3 rot;
				tf::Matrix3x3(currentTransform.getRotation()).getRPY(rot.x, rot.y, rot.z);

				//Get relevant heading variables
				currentHeading = rot.z;
				goalThrustRotation = currentTransform.getRotation();

				//Align velocity frame with the body frame
				tf::Vector3 vel;	//TODO: This probably only works when the UAV is right-side-up
				tf::vector3MsgToTF(worldVelocity.linear, vel);
				vel = tf::quatRotate(tf::createQuaternionFromYaw(-rot.z), vel);
				tf::vector3TFToMsg(vel, bodyVelocity.linear);

				if( DEBUG_MSGS ) {
					ROS_INFO("Cur Vel: [%0.2f, %0.2f, %0.2f]", bodyVelocity.linear.x, bodyVelocity.linear.y, bodyVelocity.linear.z );
					ROS_INFO("Goal Vel: [%0.2f, %0.2f, %0.2f; %0.2f]", goalVelocity.twist.linear.x, goalVelocity.twist.linear.y, goalVelocity.twist.linear.z, goalVelocity.twist.angular.z);
				}
			}
		} else {
			ROS_ERROR_ONCE( "[Timeout] No new transform information" );
		}

		//Main control loop
		//If inputs are lost at all, panic
		if( freshCommands && freshTransform ) {
			ROS_INFO_THROTTLE( 1.0, "Publishing attitude setpoints..." );

			//Calculate goal accelerations
			double Tx = vel_x_pid.step(goalVelocity.twist.linear.x, bodyVelocity.linear.x );
			double Ty = vel_y_pid.step(goalVelocity.twist.linear.y, bodyVelocity.linear.y );
			double Tz = vel_z_pid.step(goalVelocity.twist.linear.z, bodyVelocity.linear.z ) + param_throttle_mid; //TODO: Add in a hover approximation
			double goalHeading = currentHeading + ( param_yaw_rate_ff * goalVelocity.twist.angular.z * ( 1.0 / loopRate ) );// - M_PI/2;

			while( goalHeading > M_PI)
				goalHeading -= M_PI * 2;

			while( goalHeading < -M_PI)
				goalHeading += M_PI * 2;

			if( DEBUG_MSGS ) {
				ROS_INFO("[Yaw, Goal]: [%0.2f, %0.2f]", currentHeading, goalHeading );
				ROS_INFO("Goal Thrust: [%0.2f, %0.2f, %0.2f]", Tx, Ty, Tz);
			}

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
			if( gThrust.length() > param_thr_max ) {
				//If thrust_z larger than thr_max, limit throttle, set xy to 0
				if( gThrust.getZ() > param_thr_max ) {
					gThrust.setX( 0.0 );
					gThrust.setY( 0.0 );
					gThrust.setZ( param_thr_max );
					ROS_WARN_THROTTLE(0.1, "Too much thrust, scaling Z");
				} else { //The XY thrust is the cause of the over-thrust, so scale down
					double thrust_xy_max = sqrtf( ( param_thr_max * param_thr_max ) - ( gThrust.getZ() * gThrust.getZ() ) );
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
				body_z = tf::quatRotate(tf::createQuaternionFromYaw(goalHeading), body_z);

				if( DEBUG_MSGS )
					ROS_INFO( "body_z: [%0.2f, %0.2f, %0.2f]", body_z.getX(), body_z.getY(), body_z.getZ() );
			} else {
				//No thrust commanded, align with straight up
				body_z.setZ( 1.0 );
				ROS_INFO("No thrust command, keeping zero Z orientation!");
			}

			//If the thrust vector is anything at all //TODO: This can be assumed as OK?
			if (fabs(body_z.getZ()) > SIGMA) {
				//Get a vector that aligns the Y axis with the goal heading (X axis with heading + pi/2)
				tf::Vector3 yaw_r( -sin(goalHeading), cos(goalHeading), 0.0f );
				//Get the orthagonal vector to that (which will have correct pitch)
				body_x = yaw_r.cross(body_z);
				//TODO: there is a check that can be made here to make sure forward is kept when upside-down
				body_x.normalize();
			} else {
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

			if( DEBUG_MSGS ) {
				ROS_INFO( "Output Thrust: [%0.2f, %0.2f, %0.2f]", gThrust.getX(), gThrust.getY(), gThrust.getZ() );
				ROS_INFO( "Output Rotation:\n\tx: [%0.2f, %0.2f, %0.2f]\n\ty: [%0.2f, %0.2f, %0.2f]\n\tz: [%0.2f, %0.2f, %0.2f]",	body_x.getX(), body_x.getY(), body_x.getZ(),
																															body_y.getX(), body_y.getY(), body_y.getZ(),
																															body_z.getX(), body_z.getY(), body_z.getZ());
			}
		} else {
			//Prevent PID windup
			vel_x_pid.reset();
			vel_y_pid.reset();
			vel_z_pid.reset();
			//TODO: more resets here? (DOESN'T CUT THRUST
			ROS_INFO_THROTTLE( 1.0, "Publishing to hold orientation, and low thrust..." );
		}

		//TODO: When available: outputAttitude.body_rate = goalVelocity.twist.angular;
		outputAttitude.header.stamp = ros::Time::now();
		outputAttitude.header.seq++;
		tf::quaternionTFToMsg( goalThrustRotation, outputAttitude.orientation );
		outputAttitude.thrust = goalThrust;

		goalTransform.setOrigin( tf::Vector3() );
		goalTransform.setRotation( goalThrustRotation );

		tfbr.sendTransform(tf::StampedTransform(goalTransform, ros::Time::now(), "fcu/ref", "fcu/goal"));
		att_pub.publish(outputAttitude);

		//Debug Output //================================================================
		if( DEBUG_MSGS ) {
			geometry_msgs::Vector3 outputRot;
			tf::Matrix3x3(goalThrustRotation).getRPY(outputRot.x, outputRot.y, outputRot.z);

			ROS_INFO("Output (rad): [%0.2f, %0.2f, %0.2f; %0.2f]", outputRot.x, outputRot.y, outputRot.z, goalThrust);
			ROS_INFO("Output (deg): [%0.2f, %0.2f, %0.2f; %0.2f]", 180*outputRot.x/M_PI, 180*outputRot.y/M_PI, 180*outputRot.z/M_PI, goalThrust);
			ROS_INFO("--------");
		}

		//Sleep //================================================================
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

