#include "ros/ros.h"
/* This is the kinematic chain for the original OP3 robot. If your robot has the original
 configuration just uncomment this line and comment the line for the modified kinematics */
// #include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

// Our robot uses a gripper and is modified from the original
#include "mod_op3_kinematics_dynamics.h"

#include "link_data.h"
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Eigen>

using namespace robotis_op;

//OP3KinematicsDynamics* op3_kd_; // For original robot
ModOP3KinematicsDynamics* op3_kd_;

// Stores the current value for each joint, updated from the callback
std::map<std::string, double> joint_positions_;

std::map<std::string, double> joint_deltas_;
