# Control Project
### Simulation for a controller for a 2 'finger' gripper using gazebo and ros.

#### Running:
to run the gazebo simulation `roslaunch r2d2 r2d2.launch`
to run the cpp controller `rosrun r2d2 gripper_controller`
there is an rviz instance `rviz/r2d2.rviz` that will allow direct calls to the controllers. 

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

## ~~Lift Control~~
`As the simulation has changed this is no longer applicable`
> move an object to a set height and back to ground without overshoot.

## Simulation in Gazebo resources
[URDF Gazebo Simulation](https://wiki.ros.org/action/fullsearch/urdf/Tutorials/Using a URDF in Gazebo?action=fullsearch&context=180&value=linkto%3A"urdf%2FTutorials%2FUsing+a+URDF+in+Gazebo")
[Ros Control Documentation](http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros)
[Gazebo Gripper Tutorial](http://gazebosim.org/tutorials/?tut=simple_gripper)
[Gazebo Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins#Pluginsavailableingazebo_plugins)

## Current State and TODO's

There is a functional simulation *R2D2* robot that has a gripper on an arm that can be extended and retracted, in addition to having the ability to open and close the gripper. There is an outline of the gripper control ros node that can send positions (in radians) to the gripper.

Currently I am working on an assumtion that the ros topic `\joint_states` give the motor **truth** position value, and the ros topic `/r2d2_gripper_controller/command` is used to set how far the *motor* controlling the gripper rotates.

- [x] determine where the controller fits into the simulation.
- [  ] construct a controller that can close and open gripper using a PID.
  - [  ] establish an encoder for odometry calcualtions for the gripper.
- [  ] add a force sensor to the gripper.
- [  ] using the force sensor and a set *lifting force* close the gripper until a set force is reached
- [  ] update the gripper controller to account for force readings when determining the speed that the controller closes.


## Notes on force vs velocity control
based on stackoverflow comments, force based control appears to be a better method of operating joints in gazebo as setting velocity apparently can have unintended side effects that break the physics simulations in the world.
