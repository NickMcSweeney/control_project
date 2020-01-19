// General includes
#include <vector>
#include <iostream>
#include <cstring>

// ROS includes
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

// Global variables
ros::Publisher pub;

class R2D2 {
  float position_;
  float velocity_;
  
  float sensor_position_;
  float sensor_force_;

  float gripper_position_;
  float gripper_velocity_;
  float gripper_effort_;
  public:
  Gripper() {
    this->effort_ = 0;
    this->position_ = 0;
    this->velocity_ = 0;
    this->sensor_position_ = 0;
    this->sensor_force_ = 0;
    this->gripper_velocity_ = 0;
    this->gripper_position_ = 0;
    this->gripper_effort_ = 0;
  }
  void update_location(float p, float v) {
    this->position_ = p;
    this->velocity_ = v;
  }
  void update_gripper(float p, float v, float e) {
    this->gripper_position_ = p;
    this->gripper_velocity_ = v;
    this->gripper_effort_ = e;
  }
  void update_sensor(float p, float f) {
    this->sensor_position_ = p;
    this->sensor_force_ = f;
  }
};

class GripperController {
  /**
   * NodeHandle is the main access point to communications with
   * the ROS system.
   */
  ros::NodeHandle nh;
  ros::Subscriber r2d2_gripper_sub;
  ros::Subscriber r2d2_sensor_sub
  ros::Subscriber r2d2_position_sub
  ros::Publisher gripper_effort_pub;
  ros::Publisher gripper_pos_pub;

  R2D2 r2r2_state_ = R2D2();

  void gripperCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // set gripper state;
    this->r2d2_state.update_gripper(0,0,0);
  }
  void positionCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    const std::vector<std::string> &names = msg->name;
    //for(size_t i=0; i<names.size(); ++i) {
      //float p = msg->position[i];
      //float v = msg->velocity[i];
      //if (names[i] == left_gripper_.name) left_gripper_.update(p,v);
      //if (names[i] == right_gripper_.name) right_gripper_.update(p,v);
    //}
    this->r2d2_state.update_location(0,0);
  }
  void sensorCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    const std::vector<std::string> &names = msg->name;
    //for(size_t i=0; i<names.size(); ++i) {
      //float p = msg->position[i];
      //float v = msg->velocity[i];
      //if (names[i] == left_gripper_.name) left_gripper_.update(p,v);
      //if (names[i] == right_gripper_.name) right_gripper_.update(p,v);
    //}
    this->r2r2_state_.update_sensor(0,0);
  }

  void setEffort() {
    vector<double> new_pos = {0.1, 0.1};
    std_msgs::Float64MultiArray msg;

    // set up dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = new_pos.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "/r2d2_gripper_controller/command";
    
    // copy in the data
    msg.data.clear();
    msg.data.insert(msg.data.end(), new_pos.begin(), new_pos.end());

    this->gripper_effort_pub.publish(msg);
  }

  void setPosition() {
    vector<double> new_pos = {0.1, 0.1};
    std_msgs::Float64MultiArray msg;

    // set up dimensions
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = new_pos.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "/r2d2_gripper_controller/command";
    
    // copy in the data
    msg.data.clear();
    msg.data.insert(msg.data.end(), new_pos.begin(), new_pos.end());

    this->gripper_pos_pub.publish(msg);
  }
  
  public:

  // Constructor
  GripperController() {
    this->r2d2_gripper_sub = this->nh.subscribe("/r2d2/", 1000, &GripperController::gripperCallback, this);
    this->r2d2_sensor_sub = this->nh.subscribe ("/r2d2/", 1000, &GripperController::sensorCallback, this);
    this->r2d2_position_sub = this->nh.subscribe ("/r2d2/", 1000, &GripperController::positionCallback, this);
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
int main(int argc, char *argv[])  {

  // Init the connection with the ROS system  
  ros::init(argc, argv, "control_node");

  GripperController c;

  // ROS main loop
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    // Check for incomming sensor messages
    ros::spinOnce();
    loop_rate.sleep();

    c.run();
  }  
  return 0;
}
