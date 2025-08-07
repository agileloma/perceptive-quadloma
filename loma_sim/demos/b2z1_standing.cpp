/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   b2z1_standing.cpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Demo file for b2z1 standing
 * @date   June 29, 2025
 **/

#include <atomic>
#include <string>
#include <iostream>

#include <commutils/yaml/yaml_cpp_fwd.hpp>

#include "loma_sim/raisim_environment.hpp"

/**
 * Interrupt signal flag
 * This is set to true if an interrupt signal (e.g. Ctrl-c)
 * is sent to the process.
 */
std::atomic<bool> sigint{false};

/**
 * Handle the interrupt signal.
 * This enables the user to stop the program while still keeping the robot safe.
 */
void handleSigint(int )
{
    sigint = true;
}

int main(int argc, char *argv[])
{
    // Load configuration file
    std::string rel_cfg_path, abs_cfg_path;
    if (argc == 2) {
        rel_cfg_path = std::string(argv[1]);
    }
    else {
        std::cout << "Usage: ./demo /<config file within root folder>" 
                  << std::endl;
        return 1;
    }

    abs_cfg_path = ROOT_PATH + rel_cfg_path;

    YAML::Node config = YAML::LoadFile(abs_cfg_path.c_str());
    YAML::Node world_node = config["world"];
    YAML::Node robot_node = config["b2z1"];

    loma_sim::RaisimEnvironment env(world_node);

    std::shared_ptr<loma_sim::Robot> b2z1 = std::make_shared<loma_sim::Robot>(
        env.getWorld(), robot_node, ROOT_PATH, 0., 0.);
    env.addRobot(b2z1);

    env.initializeServer();

    Eigen::VectorXd desired_joint_positions(19);
    desired_joint_positions << 0.0, 0.67, -1.3, 
                               0.0, 0.67, -1.3,
                               0.0, 0.67, -1.3,
                               0.0, 0.67, -1.3,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd desired_joint_velocities(19);
    desired_joint_velocities << 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd desired_joint_efforts(19);
    double kp = 400, kd = 5;


    signal(SIGINT, handleSigint);

    while (!sigint)
    {
        env.updateStates();

        desired_joint_efforts = 
            kp * (desired_joint_positions - b2z1->getJointPositions()) + 
            kd * (desired_joint_velocities - b2z1->getJointVelocities());

        b2z1->setJointEfforts(desired_joint_efforts);

        env.step();
        sleep(0.002);
    }

    return 0;
}