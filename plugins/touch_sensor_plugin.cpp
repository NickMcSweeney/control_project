#include "ros/ros.h"

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

#include <functional>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
/// \brief An example plugin for a contact sensor.
class TouchSensorPlugin : public SensorPlugin {

public:
	/// \brief Load the sensor plugin.
	/// \param[in] _sensor Pointer to the sensor that loaded this plugin.
	/// \param[in] _sdf SDF element that describes the plugin.
	void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {
		this->sensor_ = _sensor;
		this->sensor_->SetActive(true);
		// Get the parent sensor.
		this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

		// Make sure the parent sensor is valid.
		if (!this->parentSensor) {
			gzerr << "TouchSensorPlugin requires a TouchSensor.\n";
			return;
		}

		// Connect to the sensor update event.
		this->updateConnection =
				this->parentSensor->ConnectUpdated(std::bind(&TouchSensorPlugin::OnUpdate, this));

		// Make sure the parent sensor is active.
		this->parentSensor->SetActive(true);

		if (!ros::isInitialized()) {
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
		}
		// Create our ROS node. This acts in a similar manner to the Gazebo node
		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

		// Spin up the queue helper thread.
		this->rosQueueThread = std::thread(std::bind(&TouchSensorPlugin::QueueThread, this));

		std::string pub_path = "/" + _sensor->Name() + "/";
		this->touch_pub_ = this->rosNode->advertise<geometry_msgs::Vector3>(pub_path + "contact", 1000);
		this->name_pub_ = this->rosNode->advertise<std_msgs::String>(pub_path + "name", 1000);
	}
	/// \brief Callback that receives the contact sensor's update signal.
private:
	// brief ROS helper function that processes messages
	// NOTE: not totally sure what this actually does...
	void QueueThread() {
		static const double timeout = 0.01;
		while (this->rosNode->ok()) {
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		}
	}
	void OnUpdate() {
		// Get all the contacts.
		geometry_msgs::Vector3 msg;
		std_msgs::String str_msg;
		msg.x = 0;
		msg.y = 0;
		msg.z = 0;
		str_msg.data = "";

		// int collision_count = this->parentSensor->GetCollisionCount();

		// std::string collision_name = this->parentSensor->GetCollisionName(0);

		// msgs::Contacts contacts = this->parentSensor->Contacts();

		// str_msg.data = contacts.contact_size() > 0
		//? contacts.contact(0).collision1() + " // " + contacts.contact(0).collision2()
		//: "no collision";

		// for (unsigned int i = 0; i < contacts.contact_size(); ++i) {
		// std::cout << "Collision between[" << contacts.contact(i).collision1() << "] and ["
		//<< contacts.contact(i).collision2() << "]\n";

		// for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j) {
		// std::cout << j << "  Position:" << contacts.contact(i).position(j).x() << " "
		//<< contacts.contact(i).position(j).y() << " " << contacts.contact(i).position(j).z()
		//<< "\n";
		// std::cout << "   Normal:" << contacts.contact(i).normal(j).x() << " "
		//<< contacts.contact(i).normal(j).y() << " " << contacts.contact(i).normal(j).z()
		//<< "\n";
		// std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
		//}
		//}
		this->touch_pub_.publish(msg);
		this->name_pub_.publish(str_msg);
	}

	std::unique_ptr<ros::NodeHandle> rosNode;
	ros::CallbackQueue rosQueue;
	std::thread rosQueueThread;

	ros::Publisher touch_pub_;
	ros::Publisher name_pub_;
	/// \brief Pointer to the contact sensor
	sensors::ContactSensorPtr parentSensor;
	sensors::SensorPtr sensor_;

	/// \brief Connection that maintains a link between the contact sensor's
	/// updated signal and the OnUpdate callback.
	event::ConnectionPtr updateConnection;
};
GZ_REGISTER_SENSOR_PLUGIN(TouchSensorPlugin)
} // namespace gazebo
