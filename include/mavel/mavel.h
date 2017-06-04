#include <ros/ros.h>
#include <pidController/pidController.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <string>

//PID-based position, velocity and accleration controller for a UAV
//Output calculated with normalized thrust vector and orientation.
//	Note: If either external velocity or acceleration input is used,
//	then a z body rate is sent instead of a orientation (angular
//	acceleration around the z axis are integrated).
//
//		Position Control (Optional: set timout to 0.0):
//			Reference:
//				- Pose message
//				- TF frame
//			Goal:
//				- Pose message
//				- TF frame
//			Feedback:
//				- Pose message
//				- TF frame
//		Velocity Control (Optional: set timout to 0.0):
//			Reference:
//				- Twist message
//				- TF frame (internal twist estimate)
//			Goal:
//				- Twist message
//			Feedback:
//				- Twist message
//		Acceleration control options:
//			Goal:
//				- Accel message
//			Feedback:
//				- Accel message

enum {
	HEALTH_ACTIVE = 0,
	HEALTH_TIMOUT,
	HEALTH_UNKNOWN
} mavel_data_stream_states;

template<typename streamDataT> struct mavel_data_stream {
	bool state;				//Whether the stream is reliable

	ros::Time stamp;		//Time of the last update
	ros::Duration timeout;	//How long to wait before a timeout
	double count;			//The current data count since last timeout

	string stream_topic;	//Name of the data stream (for console output)
	streamDataT data;		//Latest data from the stream
};

struct mavel_params_pid {
	double kp;
	double ki;
	double kd;
	double tau;

	string param_base_name;	//Name of parameters to query the parameter server woth
}

class Mavel {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_ping_;

		ros::Timer timer_tf_;
		ros::Timer timer_control_position_;
		ros::Timer timer_control_velocity_;
		ros::Timer timer_control_acceleration_;

		double param_rate_tf_lookups;
		double param_rate_control_position_;
		double param_rate_control_velocity_;
		double param_rate_control_acceleration_;

		bool param_external_velocity_setpoint;
		bool param_external_acceleration_setpoint;

		//Timeout in seconds
		//Required stream count is derived as 1/timeout
		double param_timeout_stream_reference_position;
		double param_timeout_stream_reference_acceleration;
		double param_timeout_stream_setpoint_position;
		double param_timeout_stream_setpoint_velocity;
		double param_timeout_stream_setpoint_acceleration;

		mavel_data_stream<geometry_msgs::PoseStamped> stream_reference_position_;
		mavel_data_stream<geometry_msgs::TwistStamped> stream_reference_velocity_;
		mavel_data_stream<geometry_msgs::PoseStamped> stream_setpoint_position_;
		mavel_data_stream<geometry_msgs::TwistStamped> stream_setpoint_velocity_;
		mavel_data_stream<geometry_msgs::AccelStamped> stream_setpoint_acceleration_;

		mavel_params_pid param_pid_position_x;
		mavel_params_pid param_pid_position_y;
		mavel_params_pid param_pid_position_z;
		mavel_params_pid param_pid_velocity_x;
		mavel_params_pid param_pid_velocity_y;
		mavel_params_pid param_pid_velocity_z;

		double integrator_body_rate_z_;

		bool param_tf_reference_position_;
		bool param_tf_reference_velocity_;
		bool param_tf_setpoint_position_;
		std::string tf_frame_world_;
		std::string tf_frame_mav_;
		std::string tf_frame_goal_;

		std::string topic_input_position_reference_;
		std::string topic_input_velocity_reference_;

		std::string topic_input_position_setpoint_;
		std::string topic_input_velocity_setpoint_;
		std::string topic_input_acceleration_setpoint_;

		std::string topic_output_position_;
		std::string topic_output_velocity_;
		std::string topic_output_acceleration_;
		std::string topic_output_attitude_;

	public:
		Mavel( void );

		~Mavel( void );

		void reference_position_cb( const geometry_msgs::PoseStamped::ConstPtr& msg_in );
		void reference_velocity_cb( const geometry_msgs::TwistStamped::ConstPtr& msg_in );
		//Handles all the TF requests as set by the params
		//Calls the reference and setpoint callbacks to handle the actual data contained
		void reference_tf_cb( const ros::TimerEvent& );

		void setpoint_position_cb( const geometry_msgs::PoseStamped::ConstPtr& msg_in );
		void setpoint_velocity_cb( const geometry_msgs::TwistStamped::ConstPtr& msg_in );
		void setpoint_accel_cb( const geometry_msgs::AccelStamped::ConstPtr& msg_in );

		//These will hanlde controller steps and publishing any feedback data
		void controller_position_cb( const ros::TimerEvent& );	//Generates a velocity setpoint
		void controller_velocity_cb( const ros::TimerEvent& );	//Generates an acceleration setpoint
		void controller_acceleration_cb( const ros::TimerEvent& );	//Generates the attitude target

	private:
		//Initializes the pid parameters for a controller
		//param_base_name should be set first (e.g. "~/pid/position/")
		void param_pid_init( mavel_params_pid* pid );

		//Initializes the stream parameters
		void stream_init( mavel_data_stream* stream );

		//Updates the stream information that new data has been recieved
		//New data should have already been added into the stream
		void stream_update( mavel_data_stream* stream );

		//Checks the stream for a timeout
		void stream_check( mavel_data_stream* stream, ros::Time time_now );
};
