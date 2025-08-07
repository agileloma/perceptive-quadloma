/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   robot.hpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Source file for Robot class
 * @date   June 30, 2025
 **/

#include "loma_sim/robot.hpp"


namespace loma_sim {

Robot::Robot(raisim::World* world, 
             const YAML::Node& robot_node,
             const std::string& root_path,
             double x_init, double y_init)
{
    // Read parameters from configuration file
    try {
        YAML::readParameter(robot_node, "urdf_path", urdf_path_);
        YAML::readParameter(robot_node, "fixed_base", fixed_base_);
        YAML::readParameter(robot_node, "base_name", base_name_);
        YAML::readParameter(robot_node, "imu_name", imu_name_);
        YAML::readParameter(robot_node, "arm_endeff_names", arm_endeff_names_);
        YAML::readParameter(robot_node, "leg_endeff_names", leg_endeff_names_);
        YAML::readParameter(robot_node, "joint_order", joint_order_);
    }
    catch (std::runtime_error& e) {
        std::cout << "[loma_sim/Robot::Robot]: "
                  << "Error reading parameter [" << e.what() << "]" 
                  << std::endl;
    }

    articulated_system_ = world->addArticulatedSystem(root_path + urdf_path_);

    for (const auto& name : arm_endeff_names_) {
        arm_endeff_index_.push_back(articulated_system_->getBodyIdx(name));
    }

    for (const auto& name : leg_endeff_names_) {
        leg_endeff_index_.push_back(articulated_system_->getBodyIdx(name));
    }

    gc_dim_ = articulated_system_->getGeneralizedCoordinateDim();
    gv_dim_ = articulated_system_->getDOF();
    num_joints_ = fixed_base_ ? gv_dim_ : gv_dim_ - 6;

    gc_ = Eigen::VectorXd::Zero(gc_dim_);
    gv_ = Eigen::VectorXd::Zero(gv_dim_);
    gf_ = Eigen::VectorXd::Zero(gv_dim_);

    gc_init_ = Eigen::VectorXd(gc_dim_);

    Eigen::Vector3d nominal_base_position;
    Eigen::Vector4d nominal_base_orientation;
    Eigen::VectorXd nominal_joint_configuration(num_joints_);
    try {
        YAML::readParameter(robot_node, 
            "nominal_base_position", nominal_base_position);
        YAML::readParameter(robot_node, 
            "nominal_base_orientation", nominal_base_orientation);
        YAML::readParameter(robot_node, 
            "nominal_joint_configuration", nominal_joint_configuration);
    }
    catch (std::runtime_error& e) {
        std::cout << "[loma_sim/Robot::Robot]: Error reading parameter ["
                  << e.what() << "]" << std::endl;
    }

    gc_init_ << nominal_base_position, 
                nominal_base_orientation, 
                nominal_joint_configuration;
    gc_init_[0] = x_init; 
    gc_init_[1] = y_init;

    gv_init_.setZero(gv_dim_);
}

void Robot::updateStates() 
{
    // Get generalized data
    articulated_system_->getState(gc_, gv_);
    gf_ = articulated_system_->getGeneralizedForce().e();
}

void Robot::setJointEfforts(const Eigen::VectorXd& efforts)
{
    if (efforts.size() == num_joints_) {
        Eigen::VectorXd desired_generalized_force(25);
        desired_generalized_force.setZero();
        desired_generalized_force.tail(num_joints_) = efforts;
        articulated_system_->setGeneralizedForce(desired_generalized_force);
    }
    else {
        throw std::runtime_error(
            "[loma_sim/Robot::setJointEfforts]: \
            Mismatch between input size and the number of actuated joints");
    }
}

Eigen::VectorXd Robot::getJointPositions() const
{
    return gc_.tail(num_joints_);;
}

Eigen::VectorXd Robot::getJointVelocities() const
{
    return gv_.tail(num_joints_);;
}

Eigen::VectorXd Robot::getJointEfforts() const
{
    return gf_.tail(num_joints_);
}

Eigen::Vector3d Robot::getBasePosition() const
{
    return gc_.head(3);
}

Eigen::Matrix3d Robot::getBaseRotation() const
{
    Eigen::Quaterniond quat(gc_.segment<4>(3));
    return quat.toRotationMatrix();
}

Eigen::Vector4d Robot::getBaseQuaternion() const
{
    return gc_.segment<4>(3);
}

Eigen::Vector3d Robot::getBaseEulerXYZ() const
{
    Eigen::Quaterniond quat(gc_.segment<4>(3));
    return quat.toRotationMatrix().eulerAngles(0, 1, 2);  // xyz
}

Eigen::Vector3d Robot::getBaseLinearVelocity() const
{
    return getBaseRotation().transpose() * gv_.segment<3>(0);
}

Eigen::Vector3d Robot::getBaseAngularVelocity() const
{
    return getBaseRotation().transpose() * gv_.segment<3>(3);
}


}  // namespace loma_sim