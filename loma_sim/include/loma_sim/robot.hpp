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
    Robot(raisim::ArticulatedSystem* articulated_system, 
          const YAML::Node& robot_node,
          int robot_id, 
          double x_init, double y_init);

private:
    raisim::ArticulatedSystem* articulated_system_{nullptr};
    int robot_id_;

    bool fixed_base_;
    std::string base_name_;
    std::string imu_name_;
    std::vector<std::string> arm_endeff_names_;
    std::vector<std::string> leg_endeff_names_;
    std::vector<std::string> joint_order_;

    std::vector<int> arm_endeff_index_;
    std::vector<int> leg_endeff_index_;

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