#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <string>

class OdomPoseHelper {
	private:
		ros::NodeHandle nh_;

		ros::Publisher pub_odom_;
		ros::Subscriber sub_pose_;

		std::string topic_odom_;
		std::string topic_pose_;
		nav_msgs::Odometry msg_odom_;

		std::string frame_world_;
		std::string frame_mav_;

		geometry_msgs::PoseStamped prev_pose_;
		tf2::Vector3 vel_l_filtered_;
		tf2::Vector3 vel_a_filtered_;

		double param_vel_filter_beta_;

	public:
		OdomPoseHelper() :
			nh_( "~" ),
			frame_world_("world"),
			frame_mav_("fcu"),
			topic_odom_("/odom"),
			topic_pose_("/pose"),
			param_vel_filter_beta_(0.2) {

			//Get parameters, or if not defined, use the defaults
			nh_.param( "frame_world", frame_world_, frame_world_ );
			nh_.param( "frame_mav", frame_mav_, frame_mav_ );
			nh_.param( "topic_odom", topic_odom_, topic_odom_ );
			nh_.param( "topic_pose", topic_pose_, topic_pose_ );
			nh_.param( "velocity_filtering", param_vel_filter_beta_, param_vel_filter_beta_ );

			msg_odom_.header.frame_id = frame_world_;
			msg_odom_.child_frame_id = frame_mav_;

			pub_odom_ = nh_.advertise<nav_msgs::Odometry>( topic_odom_, 100 );
			sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>( topic_pose_, 10, &OdomPoseHelper::pose_cb, this );

			ROS_INFO("Listening for transforms...");
		}

		~OdomPoseHelper() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}

		void pose_cb( const geometry_msgs::PoseStamped msg_in) {
			//Get the rotation from the world to body frame (for angular measurements
			tf2::Quaternion tmp_q( msg_in.pose.orientation.x,
								   msg_in.pose.orientation.y,
								   msg_in.pose.orientation.z,
								   msg_in.pose.orientation.w );

			tf2::Vector3 vel_l_world;
			tf2::Vector3 vel_a_world;
			double dt = (msg_in.header.stamp - prev_pose_.header.stamp).toSec();

			//TODO: Calculate linear velocity
			double dx = msg_in.pose.position.x - prev_pose_.pose.position.x;
			double dy = msg_in.pose.position.y - prev_pose_.pose.position.y;
			double dz = msg_in.pose.position.z - prev_pose_.pose.position.z;
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

			msg_odom_.header.stamp = msg_in.header.stamp;
			msg_odom_.pose.pose = msg_in.pose;

			msg_odom_.twist.twist.linear.x = vel_l_filtered_.x();
			msg_odom_.twist.twist.linear.y = vel_l_filtered_.y();
			msg_odom_.twist.twist.linear.z = vel_l_filtered_.z();
			msg_odom_.twist.twist.angular.x = vel_a_filtered_.x();
			msg_odom_.twist.twist.angular.y = vel_a_filtered_.y();
			msg_odom_.twist.twist.angular.z = vel_a_filtered_.z();

			pub_odom_.publish(msg_odom_);

			prev_pose_ = msg_in;
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_helper");
	OdomPoseHelper oph;

	ros::spin();

	return 0;
}
