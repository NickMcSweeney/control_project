#include "ros/ros.h"

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>

#include <functional>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include "pid_controller.h"

namespace gazebo {

class R2D2GripperPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the robot
    this->robot_ = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&R2D2ArmPlugin::OnUpdate, this));

    this->joint_gripper_ = this->robot_->GetJoint("gripper_extension");

    if (!_sdf->HasElement("update_rate")) {
      // if parameter tag does NOT exist default to 0
      this->update_rate_ = 0;
    } else {
      // if parameter tag exists, get its value
      double u_rate;
      _sdf->GetElement("update_rate")->GetValue()->Get(u_rate);
      this->update_rate_ =
          common::Time(0, common::Time::SecToNano(u_rate)).Double();
    }

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
    }
    // Create our ROS node. This acts in a similar manner to the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Point>(
            "/set_gripper_pos", 1,
            boost::bind(&R2D2ArmPlugin::cmdPosCallback, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so_pid =
        ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
            "/set_pid", 1,
            boost::bind(&R2D2ArmPlugin::cmdPIDCallback, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSub_pid = this->rosNode->subscribe(so_pid);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&R2D2ArmPlugin::QueueThread, this));

    if (this->update_rate_ > 0.0)
      this->update_period_ = 1.0 / this->update_rate_;
    else
      this->update_period_ = 0.0;

    this->last_actuator_update_ = this->robot_->GetWorld()->SimTime();
    this->joint_state_publisher_ =
        this->rosNode->advertise<sensor_msgs::JointState>("/gripper_joint_state",
                                                          1000);

    if (!this->pid_controller_set) {
      // setup pid controller

      this->gripper_position_controller_.init(&this->gripper_encoder_pos_, 1.2, 0.025, 13, 0.00002, 1.8);
      this->pos_cmd_ = 0;

      this->gripper_encoder_pos_ = 0;
      this->gripper_encoder_vel_ = 0;

      this->pid_controller_set = true;
    }
  }

private:
  // publisher for the r2d2 gripper joint state
  void publishJointState() {
    std::vector<physics::JointPtr> joints;
    joints.push_back(this->joint_gripper_);

    ros::Time current_time = ros::Time::now();
    this->joint_state_.header.stamp = current_time;
    this->joint_state_.name.resize(joints.size());
    this->joint_state_.position.resize(joints.size());
    this->joint_state_.velocity.resize(joints.size());
    this->joint_state_.effort.resize(joints.size());

    this->joint_state_.name[0] = joints[0]->GetName();
    this->joint_state_.position[0] = joints[0]->Position(0);
    this->joint_state_.velocity[0] = joints[0]->GetVelocity(0);
    this->joint_state_.effort[0] = joints[0]->GetForce(0);

    this->joint_state_publisher_.publish(this->joint_state_);
  }

  // brief ROS helper function that processes messages
  // NOTE: not totally sure what this actually does...
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  void cmdPIDCallback(const std_msgs::Float32MultiArray::ConstPtr &cmd_msg) {
    if (cmd_msg->data.size() != 3)
      return;
    this->gripper_position_controller_.setK(cmd_msg->data[0], cmd_msg->data[1], cmd_msg->data[2]);
  }

  void cmdPosCallback(const geometry_msgs::Point::ConstPtr &cmd_msg) {
    // Configure so that the position can be any value between 0 (fully retracted) and 0.375 (fully extended).
    double pos = cmd_msg->x;

    if (pos < 0) pos = 0;
    pos = pos - 0.378;

    if (pos > -0.002) pos = -0.002;
    
    this->pos_cmd_ = pos;
  }

  void UpdateArmEncoder() {
    this->step_time_ =
        (this->robot_->GetWorld()->SimTime() - this->last_encoder_update_)
            .Double();
    this->last_encoder_update_ = this->robot_->GetWorld()->SimTime();

    this->gripper_encoder_vel_ =
        (this->gripper_encoder_pos_ - this->joint_gripper_->Position(0)) /
        this->step_time_;
    this->gripper_encoder_pos_ = this->joint_gripper_->Position(0);
  }

  // Called by the world update start event
public:
  void OnUpdate() {

    this->UpdateArmEncoder();
    
    common::Time current_time = this->robot_->GetWorld()->SimTime();
    double seconds_since_last_update =
        (current_time - last_actuator_update_).Double();
    if (seconds_since_last_update > this->update_period_) {
      // Publish joint state.
      this->publishJointState();

      // run the controller
      double target_pos = this->pos_cmd_;
      this->gripper_position_controller_.update(this->joint_gripper_, target_pos, seconds_since_last_update);

      // update actuator update period.
      this->last_actuator_update_ += common::Time(this->update_period_);
    }
  }

private:
  bool pid_controller_set = false;
  PIDController gripper_position_controller_;

  physics::ModelPtr robot_;
  physics::JointPtr joint_gripper_;
  ros::Publisher joint_state_publisher_;
  sensor_msgs::JointState joint_state_;
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::Subscriber rosSub_pid;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;

  common::Time last_actuator_update_;
  common::Time last_encoder_update_;
  double step_time_;
  double update_rate_;
  double update_period_;

  double gripper_encoder_pos_;
  double gripper_encoder_vel_;
  double pos_cmd_;

private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(R2D2GripperPlugin)
}

