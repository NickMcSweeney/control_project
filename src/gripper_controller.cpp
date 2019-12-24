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

class Gripper {
  float position;
  float velocity;
  public:
  string name;
  Gripper(string n) {
    this->name = n;
    this->position = 0;
    this->velocity = 0;
  }
  void update(float p, float v) {
    this->position = p;
    this->velocity = v;
  }
  void printData() {
    cout << "position: " << this->position << "\n";
    cout << "velocity: " << this->velocity << "\n";
  }
};

class GripperController {
  /**
   * NodeHandle is the main access point to communications with
   * the ROS system.
   */
  ros::NodeHandle nh;
  ros::Subscriber gripper_pos_sub;
  ros::Publisher gripper_pos_pub;

  Gripper left_gripper_ = Gripper("left_gripper_joint");
  Gripper right_gripper_ = Gripper("right_gripper_joint");

  void positionCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    const std::vector<std::string> &names = msg->name;
    for(size_t i=0; i<names.size(); ++i) {
      float p = msg->position[i];
      float v = msg->velocity[i];
      if (names[i] == left_gripper_.name) left_gripper_.update(p,v);
      if (names[i] == right_gripper_.name) right_gripper_.update(p,v);
    }
    cout << "--- " << left_gripper_.name << " ---\n";
    left_gripper_.printData();
    cout << "--- " << right_gripper_.name << " ---\n";
    right_gripper_.printData();
    cout << "--- ------------------ ---\n";
  }

  void setPosition() {
    vector<double> new_pos = {0.0, 0.0};
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
    this->gripper_pos_sub = this->nh.subscribe("/joint_states", 1000, &GripperController::positionCallback, this);
    this->gripper_pos_pub = this->nh.advertise<std_msgs::Float64MultiArray>("/r2d2_gripper_controller/command", 1); 
  }

  void run() {
    // the actual controller runs here
    this->setPosition();
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
