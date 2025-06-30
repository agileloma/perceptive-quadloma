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

Robot::Robot(raisim::ArticulatedSystem* articulated_system, 
             const YAML::Node& robot_node,
             int robot_id, 
             double x_init, double y_init) 
    : articulated_system_(articulated_system),
      robot_id_(robot_id)
{
    // Read parameters from configuration file
    try {
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
}


}  // namespace loma_sim