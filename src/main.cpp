// General includes
//#include <vector>

// ROS includes
#include <ros/ros.h>

// Global variables
ros::Publisher pub;

class Conroller {
  /**
   * NodeHandle is the main access point to communications with
   * the ROS system.
   */
  ros::NodeHandle n;

  void Controller() {

  }

  void run() {

  }
}

//////////////////////////////////////////////////////////////
//
// Main function
//
/////////////////////////////////////////////////////////////
int main(int argc, char *argv[])  {

  // Init the connection with the ROS system  
  ros::init(argc, argv, "control_node");

  // ROS main loop
  ros::Rate loop_rate(10);
  while (ros::ok()) {

    // Check for incomming sensor messages
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}
