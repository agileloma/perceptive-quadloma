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
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

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
    env.AddRobot(robot_node, ROOT_PATH, 0.0, 0.0);
    env.InitializeServer();


    signal(SIGINT, handleSigint);

    while (!sigint)
    {

        sleep(0.002);
    }

    return 0;
}