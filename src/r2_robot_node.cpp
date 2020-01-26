// General includes
#include <cstring>
#include <iostream>
#include <vector>

// ROS includes
#include <ros/ros.h>

#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"

#include "r2_robot/pid_controller.h"

using namespace std;

// Global variables
ros::Publisher pub;

class R2State {
	float position_;
	float velocity_;

	float sensor_position_;
	float sensor_force_;
	float sensor_velocity_;

	float gripper_position_;
	float gripper_effort_;

	float arm_position_;

public:
	float obj_slip_;
	bool init;

	R2State() {
		this->obj_slip_ = 0;

		this->position_ = 0;
		this->velocity_ = 0;

		this->sensor_position_ = 0;
		this->sensor_force_ = 0;
		this->sensor_velocity_ = 0;

		this->gripper_position_ = 0;
		this->gripper_effort_ = 0;

		this->arm_position_ = 0;

		this->init = false;
	}
	void update_position(float p, float v) {
		this->position_ = p;
		this->velocity_ = v;
	}
	void update_gripper(float p, float e) {
		this->gripper_position_ = p;
		this->gripper_effort_ = e;
	}
	void update_arm(float p) { this->arm_position_ = p; }
	void update_sensor(float p) { this->sensor_position_ = p; }
	void update_sensor_v(float v) { this->sensor_velocity_ = v; }
	void update_sensor_f(float f) { this->sensor_force_ = f; }

	bool retracted_open() {
		if (this->arm_position_ <= -0.18 && this->gripper_position_ >= 0.36)
			return true;
		return false;
	}
	bool at_obj() {
		if (this->sensor_position_ - this->position_ < 0.5)
			return true;
		return false;
	}
};

class R2Controller {
	/**
	 * NodeHandle is the main access point to communications with
	 * the ROS system.
	 */
	ros::NodeHandle nh;
	ros::Subscriber gripper_sub;
	ros::Subscriber arm_sub;
	ros::Subscriber sensor_force_sub;
	ros::Subscriber sensor_pos_sub;
	ros::Subscriber position_sub;
	ros::Publisher gripper_effort_pub;
	ros::Publisher gripper_pos_pub;
	ros::Publisher arm_pos_pub;
	ros::Publisher robot_pos_pub;

	R2State robot_state_ = R2State();
	PIDController pid = PIDController(&this->robot_state_.obj_slip_, 10, 0.5, 0.5, 0, 0);

	void gripperCallback(const sensor_msgs::JointState::ConstPtr &msg) {
		// set gripper state;
		float pos = (float)(msg->position[0] + msg->position[1]) / 2;
		this->robot_state_.update_gripper(pos, 0);
	}
	void armCallback(const sensor_msgs::JointState::ConstPtr &msg) {
		// set gripper state;
		float pos = (float)msg->position[0];
		this->robot_state_.update_arm(pos);
	}
	void positionCallback(const nav_msgs::Odometry::ConstPtr &msg) {
		float pos = msg->pose.pose.position.x;
		robot_state_.update_position(pos, 0);
	}
	void sensorForceCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
		float f_x = msg->x;
		float f_y = msg->y;
		float f_z = msg->z;
		this->robot_state_.update_sensor_f(msg->y);
	}
	void sensorPosCallback(const geometry_msgs::Point::ConstPtr &msg) {
		float pos = msg->x;
		this->robot_state_.update_sensor(pos);
	}

	void setEffort(float val) {
		std_msgs::Float32 msg;
		msg.data = val;

		this->gripper_effort_pub.publish(msg);
	}

	void setGripperPosition(float val) {
		geometry_msgs::Point msg;
		msg.x = val;
		msg.y = 0;
		msg.z = 0;

		this->gripper_pos_pub.publish(msg);
	}

	void setArmPosition(float val) {
		geometry_msgs::Point msg;
		msg.x = val;
		msg.y = 0;
		msg.z = 0;

		this->arm_pos_pub.publish(msg);
	}

	void setRobotVel(float val) {
		geometry_msgs::Twist msg;
		msg.linear.x = val;
		this->robot_pos_pub.publish(msg);
	}

public:
	// Constructor
	R2Controller() {
		this->gripper_sub =
				this->nh.subscribe("/gripper/joint_state", 1000, &R2Controller::gripperCallback, this);
		this->arm_sub = this->nh.subscribe("/arm/joint_state", 1000, &R2Controller::armCallback, this);
		this->sensor_pos_sub =
				this->nh.subscribe("/psudo_sensor/pos", 1000, &R2Controller::sensorPosCallback, this);
		this->sensor_force_sub =
				this->nh.subscribe("/psudo_sensor/effort", 1000, &R2Controller::sensorForceCallback, this);
		this->position_sub = this->nh.subscribe("/r2d2_diff_drive_controller/odom", 1000,
																						&R2Controller::positionCallback, this);

		this->gripper_effort_pub = this->nh.advertise<std_msgs::Float32>("/gripper/set_force", 1000);
		this->gripper_pos_pub = this->nh.advertise<geometry_msgs::Point>("/gripper/set_pos", 1000);
		this->arm_pos_pub = this->nh.advertise<geometry_msgs::Point>("/arm/set_pos", 1000);
		this->robot_pos_pub =
				this->nh.advertise<geometry_msgs::Twist>("/r2d2_diff_drive_controller/cmd_vel", 1000);
	}

	void init() {
		// set retracted arm ang open gripper
		if (!this->robot_state_.init) {
			this->setArmPosition(-0.2);
			this->setGripperPosition(0.4);
			if (this->robot_state_.retracted_open()) {
				ROS_INFO("Initialized!");
				this->robot_state_.init = true;
			}
		}
	}
	void go_to_obj() {
		if (!this->robot_state_.at_obj()) {
			this->setRobotVel(0.1);
		} else {
			ROS_INFO("At Object");
			this->setRobotVel(0.0);
		}
	}
	void reach_for_obj() {
		if (this->robot_state_.at_obj())
			this->setArmPosition(0);
	}
	void run() {
		// the actual controller runs here
	}
};

//////////////////////////////////////////////////////////////
//
// Main function
//
/////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

	// Init the connection with the ROS system
	ros::init(argc, argv, "r2_robot");

	R2Controller c;

	ROS_INFO("STARTING");
	// ROS main loop
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		// Check for incomming sensor messages
		ros::spinOnce();
		loop_rate.sleep();

		c.init();
		c.go_to_obj();
		c.reach_for_obj();
		c.run();
	}
	return 0;
}
