#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <string>

class TFHelper {
	private:
		ros::NodeHandle nh_;

		ros::Timer timer_tf_;
		ros::Publisher pub_odom_;

		double param_tf_lookup_rate_;
		double param_tf_timeout_duration_;
		double param_tf_averaging_interval_;
		double param_vel_filter_beta_;

		std::string topic_odom_;
		nav_msgs::Odometry msg_odom_;

		std::string tf_frame_world_;
		std::string tf_frame_mav_;

		geometry_msgs::TransformStamped prev_transform_;
		tf2::Vector3 vel_l_filtered_;
		tf2::Vector3 vel_a_filtered_;

		tf2_ros::Buffer tfbf_;
		tf2_ros::TransformListener tfln_;

	public:
		TFHelper() :
			nh_( "~" ),
			tfln_(tfbf_),
			tf_frame_world_("world"),
			tf_frame_mav_("fcu"),
			topic_odom_("/odom"),
			param_tf_lookup_rate_(50.0),
			param_tf_timeout_duration_(2.0f / param_tf_lookup_rate_),
			param_tf_averaging_interval_(0.1),
			param_vel_filter_beta_(0.2) {

			//Get parameters, or if not defined, use the defaults
			nh_.param( "frame_world", tf_frame_world_, tf_frame_world_ );
			nh_.param( "frame_mav", tf_frame_mav_, tf_frame_mav_ );
			nh_.param( "topic_odom", topic_odom_, topic_odom_ );
			nh_.param( "lookup_rate", param_tf_lookup_rate_, param_tf_lookup_rate_ );
			nh_.param( "velocity_interval", param_tf_averaging_interval_, param_tf_averaging_interval_ );
			nh_.param( "velocity_filtering", param_vel_filter_beta_, param_vel_filter_beta_ );

			param_tf_timeout_duration_ = 2.0f / param_tf_lookup_rate_;
			msg_odom_.header.frame_id = tf_frame_world_;
			msg_odom_.child_frame_id = tf_frame_mav_;

			pub_odom_ = nh_.advertise<nav_msgs::Odometry>( topic_odom_, 100 );
			timer_tf_ = nh_.createTimer( ros::Duration( 1.0f / param_tf_lookup_rate_ ), &TFHelper::tf_cb, this );

			ROS_INFO("Listening for transforms...");
		}

		~TFHelper() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}
		/*
		void lookupTwist(const std::string& tracking_frame,
						 const std::string& observation_frame,
						 const std::string& reference_frame,
						 const tf2::Vector3& reference_point,
						 const std::string& reference_point_frame,
						 const ros::Time& time,
						 const ros::Duration& averaging_interval,
						 geometry_msgs::Twist& twist) {

			ros::Time latest_time, target_time;
			tf2::CompactFrameID cfid_observation = tfbf_._lookupFrameNumber(observation_frame);
			tf2::CompactFrameID cfid_tracking = tfbf_._lookupFrameNumber(tracking_frame);
			tfbf_._getLatestCommonTime(cfid_observation, cfid_tracking, latest_time, NULL);

			if (ros::Time() == time) {
				target_time = latest_time;
			} else {
				target_time = time;
			}

			ros::Time end_time = std::min(target_time + (averaging_interval * 0.5) , latest_time);
			ros::Time start_time = std::max(ros::Time().fromSec(0.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
			ros::Duration corrected_averaging_interval = end_time - start_time; //correct for the possiblity that start time was truncated above.

			geometry_msgs::TransformStamped msg_start, msg_end;
			tf2::Stamped<tf2::Transform> start, end;

			msg_start = tfbf_.lookupTransform(observation_frame, tracking_frame, start_time);
			msg_end = tfbf_.lookupTransform(observation_frame, tracking_frame, end_time);
			tf2::fromMsg(msg_start, start);
			tf2::fromMsg(msg_end, end);


			tf2::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
			tf2::Quaternion quat_temp;
			temp.getRotation(quat_temp);
			tf2::Vector3 o = start.getBasis() * quat_temp.getAxis();
			tf2Scalar ang = quat_temp.getAngle();

			double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
			double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
			double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();


			tf2::Vector3 twist_vel ((delta_x)/corrected_averaging_interval.toSec(),
									(delta_y)/corrected_averaging_interval.toSec(),
									(delta_z)/corrected_averaging_interval.toSec());
			tf2::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());


			// This is a twist w/ reference frame in observation_frame  and reference point is in the tracking_frame at the origin (at start_time)

			//correct for the position of the reference frame
			geometry_msgs::TransformStamped msg_inverse;
			tf2::Stamped<tf2::Transform> inverse;
			msg_inverse = tfbf_.lookupTransform(reference_frame, tracking_frame, target_time);
			tf2::fromMsg(msg_inverse, inverse);
			tf2::Vector3 out_rot = inverse.getBasis() * twist_rot;
			tf2::Vector3 out_vel = inverse.getBasis()* twist_vel + inverse.getOrigin().cross(out_rot);

			//Rereference the twist about a new reference point
			geometry_msgs::PointStamped rp_orig;
			geometry_msgs::PointStamped rp_desired;
			rp_orig.header.frame_id = tracking_frame;
			rp_orig.header.stamp = target_time;
			rp_orig.point.x = 0.0;
			rp_orig.point.y = 0.0;
			rp_orig.point.z = 0.0;
			rp_desired.header.frame_id = reference_point_frame;
			rp_desired.header.stamp = target_time;
			rp_desired.point.x = reference_point.x();
			rp_desired.point.y = reference_point.y();
			rp_desired.point.z = reference_point.z();

			// Start by computing the original reference point in the reference frame:
			tfbf_.transform(rp_orig, rp_orig, reference_frame);
			// convert the requrested reference point into the right frame
			tfbf_.transform(rp_desired, rp_desired, reference_frame);
			// compute the delta
			tf2::Vector3 delta = tf2::Vector3(rp_desired.point.x, rp_desired.point.y, rp_desired.point.z) - tf2::Vector3(rp_orig.point.x, rp_orig.point.y, rp_orig.point.z);
			// Correct for the change in reference point
			out_vel = out_vel + out_rot * delta;
			// out_rot unchanged

			twist.linear.x =  out_vel.x();
			twist.linear.y =  out_vel.y();
			twist.linear.z =  out_vel.z();
			twist.angular.x =  out_rot.x();
			twist.angular.y =  out_rot.y();
			twist.angular.z =  out_rot.z();
		}

		void lookupTwist(const std::string& tracking_frame,
						 const std::string& observation_frame,
						 const ros::Time& time,
						 const ros::Duration& averaging_interval,
						 geometry_msgs::Twist& twist) {
			// ref point is origin of tracking_frame, ref_frame = obs_frame
			lookupTwist(tracking_frame, observation_frame, observation_frame, tf2::Vector3(0,0,0), tracking_frame, time, averaging_interval, twist);
		}
		*/

		//TODO: v[k] = {o[k-1]}^(-1) * (p[k] - p[k-1]) / (t[k] - t[k-1]),
		//https://answers.ros.org/question/234085/how-can-the-velocity-of-a-robot-be-found-if-we-have-vicon-coordinates-available/

		void tf_cb( const ros::TimerEvent& timer) {

			if( tfbf_.canTransform(tf_frame_world_, tf_frame_mav_, timer.current_real, ros::Duration(param_tf_timeout_duration_)) ) {
				try {
					geometry_msgs::TransformStamped transform;

					transform = tfbf_.lookupTransform(tf_frame_world_, tf_frame_mav_, timer.current_real);
					/*
					lookupTwist(tf_frame_mav_,
								tf_frame_world_,
								timer.current_real,
								ros::Duration(param_tf_averaging_interval_),
								vel);
					*/

					//Get the rotation from the world to body frame (for angular measurements
					tf2::Quaternion tmp_q( transform.transform.rotation.x,
										   transform.transform.rotation.y,
										   transform.transform.rotation.z,
										   transform.transform.rotation.w );

					tf2::Vector3 vel_l_world;
					tf2::Vector3 vel_a_world;
					double dt = (transform.header.stamp - prev_transform_.header.stamp).toSec();

					//TODO: Calculate linear velocity
					double dx = transform.transform.translation.x - prev_transform_.transform.translation.x;
					double dy = transform.transform.translation.y - prev_transform_.transform.translation.y;
					double dz = transform.transform.translation.z - prev_transform_.transform.translation.z;
					vel_l_world = tf2::Vector3( dx, dy, dz ) / dt;

					//TODO: Calculate angular velocity
					//TODO: It's probably easier to just calculate val_a in the body frame
					vel_a_world = tf2::Vector3( 0.0, 0.0, 0.0 ) / dt;

					//Velocities must be in the base_link frame
					//The base_link frame is locked in roll and pitch with the world frame, but can rotate with yaw
					tf2::Quaternion rot_l_vel;
					//tf2::Quaternion rot_a_vel;
					/*
					rot_a_vel = ( tmp_q.inverse() * tf2::Quaternion( vel_a_world.x(),
																	 vel_a_world.y(),
																	 vel_a_world.z(),
																 0.0 ) ) * tmp_q;
					*/

					//Derotate the world->body quaternion
					tf2::Matrix3x3 r( tmp_q );
					tf2::Vector3 body_x;
					tf2::Vector3 body_y( r.getRow(1) );	//Use the y vector for yaw reference
					tf2::Vector3 body_z(0.0, 0.0, 1.0);

					body_x = body_y.cross(body_z);
					body_x.normalize();
					body_y = body_z.cross(body_x);
					body_y.normalize();
					r.setValue( body_x.x(), body_x.y(), body_x.z(), body_y.x(), body_y.y(), body_y.z(), body_z.x(), body_z.y(), body_z.z() );

					r.getRotation( tmp_q );	//Copy the rotation back into the quaternion

					rot_l_vel = ( tmp_q.inverse() * tf2::Quaternion( vel_l_world.x(),
																	 vel_l_world.y(),
																	 vel_l_world.z(),
																	 0.0 ) ) * tmp_q;

					tf2::Vector3 vel_l_body(rot_l_vel.x(), rot_l_vel.y(), rot_l_vel.z());
					tf2::Vector3 vel_a_body( 0.0, 0.0, 0.0 );

					//XXX Filter:lpf_tf.setOrigin( lpf_tf.getOrigin() - ( lpf_pos_beta * ( lpf_tf.getOrigin() - lpf_raw.getOrigin() ) ) );
					vel_l_filtered_ = vel_l_filtered_ - (param_vel_filter_beta_ * (vel_l_filtered_ - vel_l_body));
					vel_a_filtered_ = vel_a_filtered_ - (param_vel_filter_beta_ * (vel_a_filtered_ - vel_a_body));

					msg_odom_.header.stamp = timer.current_real;

					msg_odom_.pose.pose.position.x = transform.transform.translation.x;
					msg_odom_.pose.pose.position.y = transform.transform.translation.y;
					msg_odom_.pose.pose.position.z = transform.transform.translation.z;

					msg_odom_.pose.pose.orientation.w = transform.transform.rotation.w;
					msg_odom_.pose.pose.orientation.x = transform.transform.rotation.x;
					msg_odom_.pose.pose.orientation.y = transform.transform.rotation.y;
					msg_odom_.pose.pose.orientation.z = transform.transform.rotation.z;

					msg_odom_.twist.twist.linear.x = vel_l_filtered_.x();
					msg_odom_.twist.twist.linear.y = vel_l_filtered_.y();
					msg_odom_.twist.twist.linear.z = vel_l_filtered_.z();
					msg_odom_.twist.twist.angular.x = vel_a_filtered_.x();
					msg_odom_.twist.twist.angular.y = vel_a_filtered_.y();
					msg_odom_.twist.twist.angular.z = vel_a_filtered_.z();

					pub_odom_.publish(msg_odom_);

					prev_transform_ = transform;
				}
				catch (tf2::TransformException &ex) {
					ROS_WARN("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
			}
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_helper");
	TFHelper tfh;

	ros::spin();

	return 0;
}
