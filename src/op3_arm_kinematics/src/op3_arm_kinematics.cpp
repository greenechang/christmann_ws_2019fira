#include "ros/ros.h"
#include "op3_arm_kinematics/op3_arm_kinematics.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <eigen3/Eigen/Eigen>
#include <cstdio>
#include <map>
#include <iostream>

using namespace robotis_op;

void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg){
    for(int i = 0; i < msg->name.size(); i++){
        //std::cout << "[" << msg->name.at(i) << "]: " << msg->position.at(i) << std::endl;
        joint_positions_[msg->name.at(i)] = msg->position.at(i);
    }
}

// Performs one iteration of the inverse kinematics loop
bool calcInverseKinematicsForArm(int from, int to, Eigen::MatrixXd tar_position,
                            Eigen::MatrixXd tar_orientation, double ik_err){
    bool ik_success = false;
    bool limit_success = false;

    std::vector<int> idx = op3_kd_->findRoute(from, to);

    Eigen::MatrixXd jacobian = op3_kd_->calcJacobian(idx);

    Eigen::MatrixXd curr_position = op3_kd_->getLinkData(to)->position_;
    Eigen::MatrixXd curr_orientation = op3_kd_->getLinkData(to)->orientation_;

    Eigen::MatrixXd err = op3_kd_->calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      return ik_success;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inv = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int i = 0; i < idx.size(); i++){
        int joint_num = idx.at(i);
        std::string jointName = op3_kd_->getLinkData(i)->name_;
        joint_deltas_[jointName] = delta_angle.coeff(i);
    }

    return ik_success;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "op3_arm_ik_node");
    ros::NodeHandle nodeHandle;

    // Create the kinematics dynamics object -- for original robot configuration
    //OP3KinematicsDynamics* op3_kd_ = new OP3KinematicsDynamics(WholeBody);

    // Create the kinematics dynamics object
    op3_kd_ = new ModOP3KinematicsDynamics(WholeBody);

    // Find the route, i.e. the list of IDs going from base to end effector of
    // each arm
    // 0 is the "ID" for the base frame of the robot. 21 for the end effector
    // of right arm and 22 for left arm
    std::vector<int> rightArmRoute = op3_kd_->findRoute(29, 21);
    std::vector<int> leftArmRoute = op3_kd_->findRoute(29, 22);

    // Subscribe to the joint state topic
    ros::Subscriber sub_joint_state = nodeHandle.subscribe("/robotis/present_joint_states", 1, &joint_states_callback);
    ros::Publisher leftArmFK_pub = nodeHandle.advertise<geometry_msgs::Pose>("/op3_kinematics/left_arm_pose", 1);
    ros::Publisher rightArmFK_pub = nodeHandle.advertise<geometry_msgs::Pose>("/op3_kinematics/right_arm_pose", 1);
    ros::Publisher joint_state_pub = nodeHandle.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 1);

    ros::Rate rate(30);

    std::map<std::string, double>::iterator pos;
    while(1){


        // Iterate over IDs in each route updating with current joint angle
        int joint_id;
        for(int i = 0; i < rightArmRoute.size(); ++i){
            joint_id = rightArmRoute.at(i);
            // The "movable" joints, read from present_joint_states topic
            if (joint_id > 0 && joint_id <= 20){
                LinkData* joint_link_data = op3_kd_->getLinkData(joint_id);
                std::string joint_name = joint_link_data->name_; 

                // Update the joint link data with last angle read from the topic
                joint_link_data->joint_angle_ = joint_positions_[joint_name];
            }
        }

        for(int i = 0; i < leftArmRoute.size(); ++i){
            joint_id = leftArmRoute.at(i);
            // The "movable" joints, read from present_joint_states topic
            if (joint_id > 0 && joint_id <= 20){
                LinkData* joint_link_data = op3_kd_->getLinkData(joint_id);
                std::string joint_name = joint_link_data->name_; 

                // Update the joint link data with last angle read from the topic
                joint_link_data->joint_angle_ = joint_positions_[joint_name];
            }
        }

        op3_kd_->calcForwardKinematics(0);

        geometry_msgs::Pose leftArmPose;
        geometry_msgs::Pose rightArmPose;

        LinkData* leftArmEF_linkData = op3_kd_->getLinkData(22);
        LinkData* rightArmEF_linkData = op3_kd_->getLinkData(21);

        // Convert rotation matrices to quaternion
        Eigen::Quaterniond leftOri = robotis_framework::convertRotationToQuaternion(leftArmEF_linkData->orientation_);
        Eigen::Quaterniond rightOri = robotis_framework::convertRotationToQuaternion(rightArmEF_linkData->orientation_);

        Eigen::Vector3d leftRPY = robotis_framework::convertRotationToRPY(leftArmEF_linkData->orientation_);
        Eigen::Vector3d rightRPY = robotis_framework::convertRotationToRPY(rightArmEF_linkData->orientation_);

        //std::cout << "Roll: " << leftRPY[0] << "\nPitch: " << leftRPY[1] << "\nYaw: " << leftRPY[2] << std::endl;

        leftArmPose.position.x = leftArmEF_linkData->position_(0);
        leftArmPose.position.y = leftArmEF_linkData->position_(1);
        leftArmPose.position.z = leftArmEF_linkData->position_(2);
        leftArmPose.orientation.x = leftOri.x();
        leftArmPose.orientation.y = leftOri.y();
        leftArmPose.orientation.z = leftOri.z();
        leftArmPose.orientation.w = leftOri.w();

        rightArmPose.position.x = rightArmEF_linkData->position_(0);
        rightArmPose.position.y = rightArmEF_linkData->position_(1);
        rightArmPose.position.z = rightArmEF_linkData->position_(2);
        rightArmPose.orientation.x = rightOri.x();
        rightArmPose.orientation.y = rightOri.y();
        rightArmPose.orientation.z = rightOri.z();
        rightArmPose.orientation.w = rightOri.w();

        leftArmFK_pub.publish(leftArmPose);
        rightArmFK_pub.publish(rightArmPose);

        // Inverse Kinematics

        Eigen::Quaterniond quatTargetOri(0.753, -0.469, -0.396, -0.236);
        std::cout << quatTargetOri.w();
        Eigen::Matrix3d target_pos;
        target_pos(0) = 0.244;
        target_pos(1) = 0.170;
        target_pos(2) = 0.407;

        std::cout << "\n";
        bool ik_ret = false;
        ik_ret = calcInverseKinematicsForArm(2, 22, target_pos, quatTargetOri.toRotationMatrix(), 1e-1);
        std::cout << "ik_ret: " << ik_ret << std::endl;

        double kp = 0.005;
        std::vector<std::string> jointNames;
        std::vector<double> jointPos;
        for(int i = 0; i < leftArmRoute.size(); i++){
            joint_id = leftArmRoute.at(i);
            double currAngle = op3_kd_->getLinkData(joint_id)->joint_angle_;
            std::string jointName = op3_kd_->getLinkData(joint_id)->name_;
            double deltaAngle = joint_deltas_[jointName];
            std::cout << "[" << joint_id << "] Name: " << jointName << " currAngle: " << currAngle << " deltaAngle: " << deltaAngle << std::endl;

            double newAngle = currAngle + kp*deltaAngle;
            std::cout << "New Angle: " << newAngle << std::endl;
            if (joint_id > 0 && joint_id <= 20){
                jointNames.push_back(jointName);
                jointPos.push_back(newAngle);
            }
        }
        sensor_msgs::JointState msg;
        msg.name = jointNames;
        msg.position = jointPos;
        //joint_state_pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
