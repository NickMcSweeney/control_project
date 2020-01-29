# Control Project
### Simulation for a controller for a 2 'finger' gripper using gazebo and ros.

## [Current State]
with the adition of a calculated force value for the effort control mechanisim, the R2 robot will no longer be able to grab the target object. this is due to poor sensor input data and if/when the contact sensor is working the issue should imedeately resolve -- or at lest allow for pid tuning -- and then be resolved.

#### Building:
- `catkin_make` will build the 3 control plugins, the sensor plugin, and the ros application.
- the location of the build files need to be included in the Gazebo Path for it to be able to run
    - `export GAZEBO_PLUGIN_PATH=$HOME'/Workspace/devel/lib:'$HOME'/Workspace/build'`
- the location of the world file needs to be added to the Gazebo path
    - `export GAZEBO_RESOURCE_PATH=$HOME'/Workspace/src/r2_robot/worlds:/opt/ros/melodic/share'`

#### Running:
- to run the gazebo simulation `roslaunch r2d2 r2d2.launch`
- to run the ros automated control implementation `rosrun r2_robot r2_robot_node`

#### Goal:
+ grip an object without dropping it
+ move an object

#### Minimum
+ At least 1 use case (object) can be lifted.
+ PID based controller implemented.
+ Test performance with different objects
+ Test alternate controllers

#### Extensions
+ Higher level controller - MPC
+ Variable objects
+ Mass detection

## Gripper Control
> grasp an object between fingers and exert only the force requred to overcome slippage.

## Notes on force vs velocity control
based on stackoverflow comments, force based control appears to be a better method of operating joints in gazebo as setting velocity apparently can have unintended side effects that break the physics simulations in the world.

## Files
### src/r2_robot_node
this can be run after the simulation has started, it will move the robot to the object, reach out grip it and then move the robot (with the object) back to it's starting location. This implements a pid based controller to smoothly move the robot within range of the target object, and a pid controler to determine the force to place on the object.
### plugins/r2_arm_plugin
this is a gazebo plugin that contains a pid actuator control of the r2 robot's arm, and publishes state information, and substrives to position commands from ros.
### plugins/r2_gripper_plugin
this is a gazebo plugin that contains a pid actuator control of the r2 robot's gripper, and publishes state information, and substrives to position commands from ros.
### plugins/pseudo_sensor_plugin
this is a gazebo plugin that is intended to be attached to a target object to publish the objects state information, providing an approximation of the data avalable if an array of sensors were being used by the robot.
Note: [it is likely world force will read zero if the gripper is inputing balanced levels of force on an object and therefore the force readings are useless.]
### plugins/touch_sensor_plugin
this is a gazebo sensor plugin that is intended to publich contact sensor data. this sensor is attached to the gripper fingers. _This is only partially completed:_ currently does not publish data to ros.
### libs/pid_controller
this is a pid controller implementation that is used by the r2_robot_node for both of the pid control mechanisms.

