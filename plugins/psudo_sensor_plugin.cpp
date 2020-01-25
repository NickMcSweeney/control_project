#include "ros/ros.h"

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <functional>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

class PsudoSensorPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the robot
    this->object_ = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PsudoSensorPlugin::OnUpdate, this));

    this->object_link_ = this->object_->GetLink("phone_body");

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

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&PsudoSensorPlugin::QueueThread, this));

    if (this->update_rate_ > 0.0)
      this->update_period_ = 1.0 / this->update_rate_;
    else
      this->update_period_ = 0.0;

    this->last_actuator_update_ = this->object_->GetWorld()->SimTime();
    this->object_pos_publisher_ =
        this->rosNode->advertise<geometry_msgs::Point>("/psudo_sensor/pos",
                                                          1000);
    this->object_vel_publisher_ =
        this->rosNode->advertise<geometry_msgs::Twist>("/psudo_sensor/vel",
                                                          1000);
    this->object_effort_publisher_ = this->rosNode->advertise<geometry_msgs::Vector3>("/psudo_sensor/effort", 1000);
  }

private:
  // publisher for r2d2's psudo sensor target object state.
  void publishObjectState() {
    geometry_msgs::Point pos;
    geometry_msgs::Twist vel;
    geometry_msgs::Vector3 effort;

    ros::Time current_time = ros::Time::now();
    //pos.header.stamp = current_time;
    //vel.header.stamp = current_time;
    //effort.header.stamp = current_time;

    // Set position
    pos.x = this->pos_[0];
    pos.y = this->pos_[1];
    pos.z = this->pos_[2];

    // Set velocity
    vel.linear.x = this->vel_[0];
    vel.linear.y = this->vel_[1];
    vel.linear.z = this->vel_[2];

    // Set force
    effort.x = this->force_[0];
    effort.y = this->force_[1];
    effort.z = this->force_[2];

    // Publish state
    this->object_pos_publisher_.publish(pos);
    this->object_vel_publisher_.publish(vel);
    this->object_effort_publisher_.publish(effort);
  }

  // brief ROS helper function that processes messages
  // NOTE: not totally sure what this actually does...
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  void UpdateObjectState() {
    // set time increment;
    this->step_time_ =
        (this->object_->GetWorld()->SimTime() - this->last_encoder_update_)
            .Double();
    this->last_encoder_update_ = this->object_->GetWorld()->SimTime();
    // set current state;
    double pos[3] = {this->object_link_->WorldPose().Pos().X(), this->object_link_->WorldPose().Pos().Y(), this->object_link_->WorldPose().Pos().Z()};

    // establish velocity
    this->vel_[0] = (this->pos_[0] - pos[0])/this->step_time_;
    this->vel_[1] = (this->pos_[1] - pos[1])/this->step_time_;
    this->vel_[2] = (this->pos_[2] - pos[2])/this->step_time_;
    //this->vel_[0] = 0;
    //this->vel_[1] = 0;
    //this->vel_[2] = 0;

    // establish position.
    this->pos_[0] = pos[0];
    this->pos_[1] = pos[1];
    this->pos_[2] = pos[2];
    //this->pos_[0] = 0;
    //this->pos_[1] = 0;
    //this->pos_[2] = 0;

    // TODO: determine what effort values are to be published.
    // external force - force applied to object?
    // world frame - relative frame
    this->force_[0] = 0.01;
    this->force_[1] = 0.01;
    this->force_[2] = 0.0;
  }

  // Called by the world update start event
public:
  void OnUpdate() {

    this->UpdateObjectState();
    
    common::Time current_time = this->object_->GetWorld()->SimTime();
    double seconds_since_last_update =
        (current_time - last_actuator_update_).Double();
    if (seconds_since_last_update > this->update_period_) {
      // Publish object state.
      this->publishObjectState();
      // update actuator update period.
      this->last_actuator_update_ += common::Time(this->update_period_);
    }
  }

private:
  physics::ModelPtr object_;
  physics::LinkPtr object_link_;

  ros::Publisher object_pos_publisher_;
  ros::Publisher object_vel_publisher_;
  ros::Publisher object_effort_publisher_;

  geometry_msgs::Point object_pos_;
  geometry_msgs::Twist object_vel_;
  geometry_msgs::Vector3 object_effort_;
  
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;

  common::Time last_actuator_update_;
  common::Time last_encoder_update_;
  double step_time_;
  double update_rate_;
  double update_period_;

  double pos_ [3];
  double vel_ [3];
  double force_ [3];

private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(PsudoSensorPlugin)
}

