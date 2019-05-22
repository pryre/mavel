#include <ros/ros.h>

#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <math.h>

class OdomTripletHelper {
	private:
		ros::NodeHandle nh_;

		ros::Subscriber sub_odom_;
		ros::Publisher pub_triplet_;

	public:
		OdomTripletHelper() :
			nh_( "~" ) {

			sub_odom_ = nh_.subscribe<nav_msgs::Odometry>( "~odom", 10, &OdomTripletHelper::odom_cb, this );
			pub_triplet_ = nh_.advertise<mavros_msgs::PositionTarget>( "~triplet", 100 );

			//ROS_INFO("Listening for odometry...");
		}

		~OdomTripletHelper() {
		}

		void odom_cb( const nav_msgs::Odometry::ConstPtr& msg_in) {
			Eigen::Matrix3d R = Eigen::Quaterniond( msg_in->pose.pose.orientation.w,
													msg_in->pose.pose.orientation.x,
													msg_in->pose.pose.orientation.y,
													msg_in->pose.pose.orientation.z ).normalized().toRotationMatrix();

			Eigen::Vector3d e = R.eulerAngles(2, 1, 0);
			//www.stengel.mycpanel.princeton.edu/Quaternions.pdf
			Eigen::Matrix3d euler_conv;
			euler_conv << 1, sin(e.x())*tan(e.y()), cos(e.x())*tan(e.y()),
						  0,            cos(e.x()),           -sin(e.x()),
						  0, sin(e.x())/cos(e.y()), cos(e.x())/cos(e.y());

			Eigen::Vector3d e_r = euler_conv*Eigen::Vector3d( msg_in->twist.twist.angular.x,
															  msg_in->twist.twist.angular.y,
															  msg_in->twist.twist.angular.z );

			Eigen::Vector3d wv = R*Eigen::Vector3d( msg_in->twist.twist.linear.x,
												    msg_in->twist.twist.linear.y,
												    msg_in->twist.twist.linear.z );

			mavros_msgs::PositionTarget msg_out;
			msg_out.header = msg_in->header;
			msg_out.type_mask = msg_out.IGNORE_AFX | msg_out.IGNORE_AFY | msg_out.IGNORE_AFZ;
			msg_out.position = msg_in->pose.pose.position;
			msg_out.velocity.x = wv.x();
			msg_out.velocity.y = wv.y();
			msg_out.velocity.z = wv.z();
			msg_out.yaw = e.z();
			msg_out.yaw_rate = e_r.z();

			pub_triplet_.publish(msg_out);
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "odom_triplet_helper");
	OdomTripletHelper oth;

	ros::spin();

	return 0;
}
