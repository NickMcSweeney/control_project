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

	float last_time_;

public:
	float obj_slip_;
	PIDController force_control = PIDController(&this->gripper_effort_, 1.5, 0.0005, 0.01, 0.0, 0.0);
	PIDController drive_control = PIDController(&this->position_, 1.0, 0.005, 0.26, 0.1, 0.07);

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

		this->last_time_ = ros::Time::now().toSec();
	}
	float mass() { return 0.5; }
	float acceleration() { return this->obj_slip_; }
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
	void update_sensor_v(float v) {
		float dt = this->last_time_ - ros::Time::now().toSec();
		this->last_time_ = ros::Time::now().toSec();
		this->obj_slip_ = (this->sensor_velocity_ - v) / dt;
		this->sensor_velocity_ = v;
	}
	void update_sensor_f(float f) { this->sensor_force_ = f; }

	float sensor_force() { return this->sensor_force_ > 0 ? this->sensor_force_ : 0; }
	float object_pose() { return this->sensor_position_ - 0.46; }

	bool retracted_open() {
		if (this->arm_position_ <= -0.18 && this->gripper_position_ >= 0.4)
			return true;
		return false;
	}
	bool extended_open() {
		ROS_INFO("[pos arm] %f [pos grip] %f", this->arm_position_, this->gripper_position_);
		if (this->arm_position_ >= -0.08 && this->gripper_position_ >= 0.3)
			return true;
		return false;
	}
	bool at_obj() {
		if (this->sensor_position_ - this->position_ < 0.6)
			return true;
		return false;
	}
	bool at_home() {
		if (this->position_ < 0.5)
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

	enum states { ready, initial, moving, target, gripping, home, finished };
	states state_;
	states next_;

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
		this->robot_state_.update_sensor_f(f_x + f_y);
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

		state_ = ready;
		next_ = initial;
	}

	void update_state(states new_next) {
		this->state_ = this->next_;
		this->next_ = new_next;
	}

	void init() {
		// set retracted arm ang open gripper
		this->setArmPosition(-0.36);
		this->setGripperPosition(0.48);
		if (this->robot_state_.retracted_open()) {
			ROS_INFO("Initialized!");
			this->update_state(target);
		}
	}
	void go_to_obj(float dt) {
		float c = this->robot_state_.drive_control.update(this->robot_state_.object_pose(), dt);
		this->setRobotVel(c);
		if (this->robot_state_.at_obj())
			this->update_state(gripping);
	}
	void reach_for_obj() {
		this->setArmPosition(0);
		// this->setGripperPosition(0.32);
		if (this->robot_state_.at_obj() & this->robot_state_.extended_open()) {
			ROS_INFO("In position to grab object!");
			this->update_state(home);
		}
	}
	void grab_object(float dt) {
		// this is where using a controller happens...
		ROS_INFO("the force on the object is: %f", this->robot_state_.sensor_force());
		float force = this->robot_state_.mass() * (9.8 + this->robot_state_.acceleration()) / 2.0;
		ROS_INFO("this target force is %f", force);
		float c = this->robot_state_.force_control.update(force, dt);
		// prevent a negative force on the object, and divide in half to account for the 2 sources of
		// force
		c = c < 0 ? 0 : c / 2;
		this->setEffort(-c);
		if (this->robot_state_.sensor_force() >= force)
			this->update_state(home);
	}
	void return_home(float dt) {
		this->grab_object(dt);

		float c = this->robot_state_.drive_control.update(0.5, dt);
		if (c > 0)
			c = 0;
		this->setRobotVel(c);
		if (this->robot_state_.at_home())
			this->update_state(finished);
	}
	void run(float dt) {
		// the actual controller runs here
		switch (this->state_) {
		case initial:
			ROS_INFO("INITIAL");
			this->init();
			break;
		case moving:
			ROS_INFO("MOVING");
			this->go_to_obj(dt);
			break;
		case target:
			ROS_INFO("TARGET");
			this->reach_for_obj();
			break;
		case gripping:
			ROS_INFO("GRIPPING");
			this->grab_object(dt);
			break;
		case home:
			ROS_INFO("HOME");
			this->return_home(dt);
			break;
		case ready:
			ROS_INFO("READY");
			this->update_state(moving);
			break;
		case finished:
			ROS_INFO("FINISHED");
			break;
		}
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
	float current_time = ros::Time::now().toSec();
	float last_time = ros::Time::now().toSec();
	float delta_time = current_time - last_time;
	// ROS main loop
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		// update time
		last_time = current_time;
		current_time = ros::Time::now().toSec();
		delta_time = current_time - last_time;

		// run robot
		c.run(delta_time);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
