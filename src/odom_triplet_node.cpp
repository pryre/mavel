#include <ros/ros.h>

#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <math.h>

class OdomTripletHelper {
	private:
		ros::NodeHandle nhp_;

		ros::Subscriber sub_odom_;
		ros::Publisher pub_triplet_;

		ros::Time time_last_;
		Eigen::Vector3d wv_last_;
		Eigen::Vector3d wa_last_;
		double wa_lpf_filter_;

	public:
		OdomTripletHelper() :
			nhp_( "~" ),
			time_last_(0),
			wa_lpf_filter_(0.1) {
			wv_last_ = Eigen::Vector3d::Zero();

			sub_odom_ = nhp_.subscribe<nav_msgs::Odometry>( "odom", 10, &OdomTripletHelper::odom_cb, this );
			pub_triplet_ = nhp_.advertise<mavros_msgs::PositionTarget>( "triplet", 100 );
		}

		~OdomTripletHelper() {
		}

		void odom_cb( const nav_msgs::Odometry::ConstPtr& msg_in) {
			Eigen::Quaterniond q = Eigen::Quaterniond( msg_in->pose.pose.orientation.w,
													   msg_in->pose.pose.orientation.x,
													   msg_in->pose.pose.orientation.y,
													   msg_in->pose.pose.orientation.z ).normalized();
			Eigen::Matrix3d R = q.toRotationMatrix();

			Eigen::Vector3d eraw = R.eulerAngles(2, 1, 0);
			Eigen::Vector3d e(eraw(2), eraw(1), eraw(0));	//eraw is given back as YPR, but we want to have it refered to using x(),y(),z()

			//https://ocw.mit.edu/courses/mechanical-engineering/2-017j-design-of-electromechanical-robotic-systems-fall-2009/course-text/MIT2_017JF09_ch09.pdf
			//pg.71
			Eigen::Matrix3d euler_conv;
			/*
			euler_conv << 1, sin(e.x())*tan(e.y()), cos(e.x())*tan(e.y()),
						  0,            cos(e.x()),           -sin(e.x()),
						  0, sin(e.x())/cos(e.y()), cos(e.x())/cos(e.y());
			*/
			euler_conv << 1, sin(e.z())*tan(e.y()), cos(e.z())*tan(e.y()),
						  0,            cos(e.z()),           -sin(e.z()),
						  0, sin(e.z())/cos(e.y()), cos(e.z())/cos(e.y());

			//Calculate euler rates
			Eigen::Vector3d e_r = euler_conv*Eigen::Vector3d( msg_in->twist.twist.angular.x,
															  msg_in->twist.twist.angular.y,
															  msg_in->twist.twist.angular.z );

			//Calculate world velocity
			Eigen::Vector3d wv = R.transpose()*Eigen::Vector3d( msg_in->twist.twist.linear.x,
																msg_in->twist.twist.linear.y,
																msg_in->twist.twist.linear.z );

			//Calculate world acceleration (if requirements met)
			Eigen::Vector3d wa = Eigen::Vector3d::Zero();
			double dt = 0.0;
			//See if time reset
			if( time_last_ > msg_in->header.stamp ) {
				time_last_ = ros::Time(0);
			}

			if( time_last_ > ros::Time(0) ) {
				dt = (msg_in->header.stamp - time_last_).toSec();

				//Only bother caclualting dirty derivative if we have a semi-decent update rate
				if( ( dt < 0.1) && ( dt > 0.0 ) ) {
					Eigen::Vector3d wa_new = (wv - wv_last_) / dt;
					wa = wa_lpf_filter_*wa_new + (1.0 - wa_lpf_filter_)*wa_last_;
				}

			}

			time_last_ = msg_in->header.stamp;
			wv_last_ = wv;
			wa_last_ = wa;

			mavros_msgs::PositionTarget msg_out;
			msg_out.header = msg_in->header;
			if(dt <= 0.0)
				msg_out.type_mask = msg_out.IGNORE_AFX | msg_out.IGNORE_AFY | msg_out.IGNORE_AFZ;
			msg_out.position = msg_in->pose.pose.position;
			msg_out.velocity.x = wv.x();
			msg_out.velocity.y = wv.y();
			msg_out.velocity.z = wv.z();
			msg_out.acceleration_or_force.x = wa.x();
			msg_out.acceleration_or_force.y = wa.y();
			msg_out.acceleration_or_force.z = wa.z();
			msg_out.yaw = yaw_from_quaternion(q);	//Use a clean value rather than the euler calculated method (which can flip randomly)
			msg_out.yaw_rate = e_r.z();

			pub_triplet_.publish(msg_out);
		}

		double yaw_from_quaternion( const Eigen::Quaterniond &q ) {
			double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
			double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
			return atan2(siny, cosy);
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "odom_triplet_helper");
	OdomTripletHelper oth;

	ros::spin();

	return 0;
}
