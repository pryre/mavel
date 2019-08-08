#include <ros/ros.h>

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Dense>
#include <math.h>

class MultiTeleopNode {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		ros::Publisher pub_triplet_;

		ros::Subscriber sub_pose_;
		ros::Subscriber sub_move_;
		ros::Subscriber sub_twist_;
		ros::Subscriber sub_receed_;

	public:
		MultiTeleopNode() :
			nh_(),
			nhp_("~") {

			sub_pose_ = nhp_.subscribe<geometry_msgs::PoseStamped>( "input/pose", 10, &MultiTeleopNode::pose_cb, this );
			sub_move_ = nhp_.subscribe<geometry_msgs::PoseStamped>( "input/pose_ground", 10, &MultiTeleopNode::ground_cb, this );
			sub_twist_ = nhp_.subscribe<geometry_msgs::TwistStamped>( "input/twist", 10, &MultiTeleopNode::twist_cb, this );
			sub_receed_ = nhp_.subscribe<geometry_msgs::TwistStamped>( "input/twist_receeding_horizon", 10, &MultiTeleopNode::receed_cb, this );

			pub_triplet_ = nhp_.advertise<mavros_msgs::PositionTarget>( "triplet", 100 );
		}

		~MultiTeleopNode() {
		}

		double yaw_from_quaternion(const geometry_msgs::Quaternion& q) {
			Eigen::Matrix3d r = Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
			return atan2(r(1,0),r(0,0));
		}

		void pose_handler( const geometry_msgs::PoseStamped& pose, const uint8_t& coordinate_frame ) {
			mavros_msgs::PositionTarget msg_out;
			msg_out.header = pose.header;
			msg_out.coordinate_frame = coordinate_frame;
			msg_out.type_mask = msg_out.IGNORE_AFX | msg_out.IGNORE_AFY | msg_out.IGNORE_AFZ | msg_out.IGNORE_VX | msg_out.IGNORE_VY | msg_out.IGNORE_VZ | msg_out.IGNORE_YAW_RATE;
			msg_out.position = pose.pose.position;
			msg_out.yaw = yaw_from_quaternion(pose.pose.orientation);

			pub_triplet_.publish(msg_out);
		}

		void pose_cb( const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
			pose_handler(*msg_in, mavros_msgs::PositionTarget::FRAME_LOCAL_NED);
		}

		void ground_cb( const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
			geometry_msgs::PoseStamped pose = *msg_in;
			pose.pose.position.z = 1.0;
			pose_handler(pose, mavros_msgs::PositionTarget::FRAME_LOCAL_NED);
		}

		void twist_cb( const geometry_msgs::TwistStamped::ConstPtr& msg_in) {
			mavros_msgs::PositionTarget msg_out;
			msg_out.header = msg_in->header;
			msg_out.coordinate_frame = msg_out.FRAME_BODY_OFFSET_NED;
			msg_out.type_mask = msg_out.IGNORE_AFX | msg_out.IGNORE_AFY | msg_out.IGNORE_AFZ;
			msg_out.velocity = msg_in->twist.linear;
			msg_out.yaw_rate = msg_in->twist.angular.z;

			pub_triplet_.publish(msg_out);
		}

		void receed_cb( const geometry_msgs::TwistStamped::ConstPtr& msg_in) {
			/*
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
			*/
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "multi_teleop_node");
	MultiTeleopNode oth;

	ros::spin();

	return 0;
}
