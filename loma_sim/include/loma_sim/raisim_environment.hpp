/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   raisim_environment.hpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Header file for RaisimEnvironment class
 * @date   June 27, 2025
 **/

#pragma once

#include <memory>
#include <vector>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <commutils/yaml/yaml_cpp_fwd.hpp>

#include "loma_sim/robot.hpp"


namespace loma_sim {

class RaisimEnvironment
{
public:
    explicit RaisimEnvironment(const YAML::Node& world_node);

    /**
     * @brief Initializes the Raisim server for visualization.
     * @details Must have added at least one robot before calling. 
     *          Launches the visualizer server.
     */
    void InitializeServer();

    void AddRobot(const YAML::Node& robot_node, 
                  const std::string& root_path,
                  double x_init = 0., double y_init = 0.);

private:
    // raisim world object
    std::unique_ptr<raisim::World> world_;

    // raisim server for visualization
    std::unique_ptr<raisim::RaisimServer> server_;

    /// List of robot objects
    std::vector<std::unique_ptr<Robot>> robots_;  

    // Pointer to current height map
    raisim::HeightMap* height_map_{nullptr};  

    // Simulation time step (seconds)
    double time_step_{0.001}; 

    // Gravity (m/s^2)
    double gravity_{9.81}; 

    // Is tracking camera active
    bool tracking_camera_{false};  
    // Is camera in free mode
    bool free_camera_{false}; 
};

}  // namespace loma_sim