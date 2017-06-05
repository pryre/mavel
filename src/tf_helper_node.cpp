#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <string>

geometry_msgs::PoseStamped TFToPoseStamped( const geometry_msgs::TransformStamped msg_in ) {
	geometry_msgs::PoseStamped msg_out;

	msg_out.header = msg_in.header;

	msg_out.pose.position.x = msg_in.transform.translation.x;
	msg_out.pose.position.y = msg_in.transform.translation.y;
	msg_out.pose.position.z = msg_in.transform.translation.z;

	msg_out.pose.orientation.w = msg_in.transform.rotation.w;
	msg_out.pose.orientation.x = msg_in.transform.rotation.x;
	msg_out.pose.orientation.y = msg_in.transform.rotation.y;
	msg_out.pose.orientation.z = msg_in.transform.rotation.z;

	return msg_out;
}

class TFHelper {
	private:
		ros::NodeHandle nh_;

		ros::Timer timer_tf_;

		ros::Publisher pub_reference_position_;
		ros::Publisher pub_reference_velocity_;
		ros::Publisher pub_setpoint_position_;

		double param_rate_tf_lookups_;
		double param_duration_tf_timeout_;

		bool param_tf_reference_position_;
		bool param_tf_reference_velocity_;
		bool param_tf_setpoint_position_;

		std::string tf_frame_world_;
		std::string tf_frame_mav_;
		std::string tf_frame_goal_;

		std::string topic_reference_position_;
		std::string topic_reference_velocity_;
		std::string topic_setpoint_position_;

	public:
		TFHelper() :
			nh_( "~" ) {

			//Get parameters, or if not defined, use the defaults
			//nh_.param( "rate_ping", rate_ping_, rate_ping_ );
			//nh_.param( "topic_output_ping", topic_output_ping_, topic_output_ping_ );

			pub_reference_position_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_reference_position_, 100 );
			pub_reference_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>( topic_reference_velocity_, 100 );
			pub_setpoint_position_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_setpoint_position_, 100 );

			//If we are using TF at all
			param_duration_tf_timeout_ = 2.0f / param_rate_tf_lookups_;

			timer_tf_ = nh_.createTimer( ros::Duration( 1.0f / param_rate_tf_lookups_ ), &TFHelper::tf_cb, this );

			ROS_INFO("Listenning for transforms...");
		}

		~TFHelper() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}


	void tf_cb( const ros::TimerEvent& timer) {
		//If we are using TF to get velocity estimate reference
		if( param_tf_reference_velocity_ ) {
			try {
				//listener lookupTwist
				geometry_msgs::TwistStamped vel_ref;
				//tf_pos_ref = tfbuffer_.lookupTransform( tf_frame_goal_, tf_frame_world_, timer.current_real, ros::Duration( param_duration_tf_timeout_ ) );
				//this->reference_position_cb( TFToPoseStamped( tf_pos_ref ) );
				pub_reference_velocity_.publish(vel_ref);
			}
			catch( tf2::TransformException &ex ) {
				ROS_WARN( "%s",ex.what() );
			}
		}

		//If we are using TF to get position reference
		if( param_tf_reference_position_ ) {
			try {
				//listener lookupTransform
				geometry_msgs::PoseStamped pos_ref;
				//geometry_msgs::TransformStamped tf_pos_ref;
				//tf_pos_ref = tfbuffer_.lookupTransform( tf_frame_goal_, tf_frame_world_, timer.current_real, ros::Duration( param_duration_tf_timeout_ ) );
				//this->reference_position_cb( TFToPoseStamped( tf_pos_ref ) );
				pub_reference_position_.publish(pos_ref);
			}
			catch( tf2::TransformException &ex ) {
				ROS_WARN( "%s",ex.what() );
			}
		}

		//If we are using TF to get position setpoint
		if( param_tf_setpoint_position_ ) {
			try {
				//listener lookupTransform
				geometry_msgs::PoseStamped pos_sp;
				//geometry_msgs::TransformStamped tf_pos_sp;
				//tf_pos_sp = tfbuffer_.lookupTransform( tf_frame_goal_, tf_frame_world_, timer.current_real, ros::Duration( param_duration_tf_timeout_ ) );
				//this->setpoint_position_cb( TFToPoseStamped( tf_pos_sp ) );
				pub_reference_position_.publish(pos_sp);
			}
			catch( tf2::TransformException &ex ) {
				ROS_WARN( "%s",ex.what() );
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
