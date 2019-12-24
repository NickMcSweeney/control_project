# Control Project
### Simulation for a controller for a 2 'finger' gripper using gazebo and ros.

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

## Lift Control
> move an object to a set height and back to ground without overshoot.

## Simulation in Gazebo resources
[URDF Gazebo Simulation](https://wiki.ros.org/action/fullsearch/urdf/Tutorials/Using a URDF in Gazebo?action=fullsearch&context=180&value=linkto%3A"urdf%2FTutorials%2FUsing+a+URDF+in+Gazebo")
[Ros Control Documentation](http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros)
[Gazebo Gripper Tutorial](http://gazebosim.org/tutorials/?tut=simple_gripper)
[Gazebo Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins#Pluginsavailableingazebo_plugins)
