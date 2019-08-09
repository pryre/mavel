#include <ros/ros.h>

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <dynamic_reconfigure/server.h>
#include <mavel/MultiTeleopParamsConfig.h>

#include <eigen3/Eigen/Dense>
#include <math.h>

class MultiTeleopNode {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		ros::Timer timer_;
		ros::Publisher pub_triplet_;

		ros::Subscriber sub_pose_;
		ros::Subscriber sub_move_;
		ros::Subscriber sub_twist_;
		ros::Subscriber sub_receed_;

		mavros_msgs::PositionTarget msg_out_;

		double param_move_height_;
		double param_output_rate_;

		dynamic_reconfigure::Server<mavel::MultiTeleopParamsConfig> dyncfg_settings_;

	public:
		MultiTeleopNode() :
			nh_(),
			nhp_("~"),
			dyncfg_settings_(ros::NodeHandle(nhp_)),
			param_move_height_(1.0),
			param_output_rate_(0.0) {

			// Prep the timer for setup through dynamic reconfigure
			timer_ = nhp_.createTimer( ros::Duration(0), &MultiTeleopNode::timer_cb, this );

			//Dynamic Reconfigure
			dyncfg_settings_.setCallback(boost::bind(&MultiTeleopNode::callback_cfg_settings, this, _1, _2));

			sub_pose_ = nhp_.subscribe<geometry_msgs::PoseStamped>( "input/pose", 10, &MultiTeleopNode::pose_cb, this );
			sub_move_ = nhp_.subscribe<geometry_msgs::PoseStamped>( "input/pose_ground", 10, &MultiTeleopNode::ground_cb, this );
			sub_twist_ = nhp_.subscribe<geometry_msgs::TwistStamped>( "input/twist", 10, &MultiTeleopNode::twist_cb, this );
			sub_receed_ = nhp_.subscribe<geometry_msgs::TwistStamped>( "input/twist_receeding_horizon", 10, &MultiTeleopNode::receed_cb, this );

			pub_triplet_ = nhp_.advertise<mavros_msgs::PositionTarget>( "triplet", 100 );
		}

		~MultiTeleopNode() {
		}

		void callback_cfg_settings( mavel::MultiTeleopParamsConfig &config, uint32_t level ) {
			param_move_height_ = config.pose_ground_height;
			param_output_rate_ = config.output_rate;

			if(param_output_rate_ > 0.0) {
				timer_.setPeriod(ros::Duration(1.0 / param_output_rate_));
				timer_.start();
			} else {
				timer_.stop();
			}
		}

		void timer_cb( const ros::TimerEvent& te ) {
			//Only send it out if the latest recieved message had a valid stamp
			//If an invalid message is recieved, then it will cancel sending.
			if( msg_out_.header.stamp > ros::Time(0) ) {
				msg_out_.header.stamp = ros::Time::now();
				pub_triplet_.publish(msg_out_);
			}
		}

		double yaw_from_quaternion(const geometry_msgs::Quaternion& q) {
			Eigen::Matrix3d r = Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
			return atan2(r(1,0),r(0,0));
		}

		void pose_handler( const geometry_msgs::PoseStamped& pose, const uint8_t& coordinate_frame ) {
			msg_out_.header = pose.header;
			msg_out_.coordinate_frame = coordinate_frame;
			msg_out_.type_mask = msg_out_.IGNORE_AFX | msg_out_.IGNORE_AFY | msg_out_.IGNORE_AFZ | msg_out_.IGNORE_VX | msg_out_.IGNORE_VY | msg_out_.IGNORE_VZ | msg_out_.IGNORE_YAW_RATE;
			msg_out_.position = pose.pose.position;
			msg_out_.yaw = yaw_from_quaternion(pose.pose.orientation);

			if(param_output_rate_ <= 0.0) {
				pub_triplet_.publish(msg_out_);
			}
		}

		void pose_cb( const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
			pose_handler(*msg_in, mavros_msgs::PositionTarget::FRAME_LOCAL_NED);
		}

		void ground_cb( const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
			geometry_msgs::PoseStamped pose = *msg_in;
			pose.pose.position.z = param_move_height_;
			pose_handler(pose, mavros_msgs::PositionTarget::FRAME_LOCAL_NED);
		}

		void twist_cb( const geometry_msgs::TwistStamped::ConstPtr& msg_in) {
			msg_out_.header = msg_in->header;
			msg_out_.coordinate_frame = msg_out_.FRAME_BODY_OFFSET_NED;
			msg_out_.type_mask = msg_out_.IGNORE_AFX | msg_out_.IGNORE_AFY | msg_out_.IGNORE_AFZ;
			msg_out_.velocity = msg_in->twist.linear;
			msg_out_.yaw_rate = msg_in->twist.angular.z;

			if(param_output_rate_ <= 0.0) {
				pub_triplet_.publish(msg_out_);
			}
		}

		void receed_cb( const geometry_msgs::TwistStamped::ConstPtr& msg_in) {
			/*
			msg_out_.header = msg_in->header;
			msg_out_.type_mask = msg_out_.IGNORE_AFX | msg_out_.IGNORE_AFY | msg_out_.IGNORE_AFZ;
			msg_out_.position = msg_in->pose.pose.position;
			msg_out_.velocity.x = wv.x();
			msg_out_.velocity.y = wv.y();
			msg_out_.velocity.z = wv.z();
			msg_out_.yaw = e.z();
			msg_out_.yaw_rate = e_r.z();

			pub_triplet_.publish(msg_out_);
			*/
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "multi_teleop_node");
	MultiTeleopNode oth;

	ros::spin();

	return 0;
}
