#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/JointState.h>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
  class SimpleGripperPlugin : public ModelPlugin {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    	printf("Hello World!\n");
			// Store the pointer to the model
			this->model = _parent;

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			//this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SimpleGripperPlugin::OnUpdate, this));

			//this->gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "R2D2Arm"));

			//this->joint_arm_ = this->model->GetJoint("gripper_extension");


			// Initialize ros, if it has not already bee initialized.
       //if (!ros::isInitialized()) {
				 //int argc = 0;
				 //char **argv = NULL;
				 //ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
			//}
			//// Create our ROS node. This acts in a similar manner to the Gazebo node
			//this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
			
      //// Create a named topic, and subscribe to it.
			//ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
                    //"/simple_gripper/vel_cmd",
                    //1,
                    //boost::bind(&SimpleGripperPlugin::OnRosMsg, this, _1),
                    //ros::VoidPtr(), &this->rosQueue);
      //this->rosSub = this->rosNode->subscribe(so);

      //// Spin up the queue helper thread.
      //this->rosQueueThread = std::thread(std::bind(&SimpleGripperPlugin::QueueThread, this));


			//this->joint_state_publisher_ = this->rosNode->advertise<sensor_msgs::JointState> ( "/arm_joint_state", 1000  );
		}

		//private: void publishJointState() {
			//std::vector<physics::JointPtr> joints;
			//joints.push_back ( this->joint_arm_  );

			//ros::Time current_time = ros::Time::now();
			//this->joint_state_.header.stamp = current_time;
			//this->joint_state_.name.resize ( joints.size()  );
			//this->joint_state_.position.resize ( joints.size()  );
			//this->joint_state_.velocity.resize ( joints.size()  );
			//this->joint_state_.effort.resize ( joints.size()  );

			//this->joint_state_publisher_.publish(this->joint_state_);
		//}

    //public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg) {
      //this->SetVelocity(_msg->data);
    //}

		//// \brief ROS helper function that processes messages
		//private: void QueueThread() {
			//static const double timeout = 0.01;
			//while (this->rosNode->ok()) {
				//this->rosQueue.callAvailable(ros::WallDuration(timeout));
			//}
		//}

    //public: void SetVelocity(const double &_vel) {
      //// Set the joint's target velocity.
			//this->model->GetJointController()->SetVelocityTarget(this->joint_arm_->GetScopedName(), _vel);
		//}

		// Called by the world update start event
		public: void OnUpdate() {
			// Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(3, 0, 0));
			//this->publishJointState();
		}

		// Pointer to the model
		private: 
			physics::ModelPtr model;
			physics::JointPtr joint_arm_;
			ros::Publisher joint_state_publisher_;
			sensor_msgs::JointState joint_state_;

			std::unique_ptr<ros::NodeHandle> rosNode;
			ros::Subscriber rosSub;
			ros::CallbackQueue rosQueue;
			std::thread rosQueueThread;
			
		
		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;

  };

	// Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SimpleGripperPlugin)
}
