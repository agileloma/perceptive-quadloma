/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   robot.hpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Header file for Robot class
 * @date   June 30, 2025
 **/

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <raisim/World.hpp>

#include <commutils/yaml/yaml_cpp_fwd.hpp>


namespace loma_sim {

class Robot
{
public:
    Robot(raisim::World* world,
          const YAML::Node& robot_node,
          const std::string& root_path,
          double x_init = 0., double y_init = 0.);

    // Reset and update robot states
    void updateStates();


    raisim::ArticulatedSystem* getArticulatedSystem() const { 
        return articulated_system_;
    };

    void setJointEfforts(const Eigen::VectorXd& efforts);

    Eigen::VectorXd getJointPositions() const;
    Eigen::VectorXd getJointVelocities() const;
    Eigen::VectorXd getJointEfforts() const;

    Eigen::Vector3d getBasePosition() const;
    Eigen::Matrix3d getBaseRotation() const;
    Eigen::Vector4d getBaseQuaternion() const;
    Eigen::Vector3d getBaseEulerXYZ() const;
    Eigen::Vector3d getBaseLinearVelocity() const;
    Eigen::Vector3d getBaseAngularVelocity() const;

private:
    // Object and its id
    raisim::ArticulatedSystem* articulated_system_{nullptr};
    int robot_id_;

    std::string urdf_path_;

    // Robot properties
    bool fixed_base_;
    std::string base_name_;
    std::string imu_name_;
    std::vector<std::string> arm_endeff_names_;
    std::vector<std::string> leg_endeff_names_;
    std::vector<std::string> joint_order_;

    std::vector<int> arm_endeff_index_;
    std::vector<int> leg_endeff_index_;

    // Generalized states
    int gc_dim_;
    int gv_dim_;
    int num_joints_;

    Eigen::VectorXd gc_;
    Eigen::VectorXd gv_;
    Eigen::VectorXd gf_;
    Eigen::VectorXd gc_init_;
    Eigen::VectorXd gv_init_;
};

}  // namespace loma_sim