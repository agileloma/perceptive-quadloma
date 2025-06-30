/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   raisim_environment.cpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Source file for RaisimEnvironment class
 * @date   June 27, 2025
 **/

#include "loma_sim/raisim_environment.hpp"


namespace loma_sim {

RaisimEnvironment::RaisimEnvironment(const YAML::Node& world_node) 
{
    // Read parameters from configuration file
    try {
        YAML::readParameter(world_node, "time_step", time_step_);
        YAML::readParameter(world_node, "gravity", gravity_);
    }
    catch (std::runtime_error& e) {
        std::cout << "[loma_sim/RaisimEnvironment::RaisimEnvironment]: "
                  << "Error reading parameter [" << e.what() << "]" 
                  << std::endl;
    }

    // Set up simulation world
    world_ = std::make_unique<raisim::World>();
    world_->setTimeStep(time_step_);
    world_->setGravity(raisim::Vec<3>{0, 0, -gravity_});
    world_->addGround();

    // Initialize default terrain
    raisim::TerrainProperties terr;
    terr.frequency = 0;
    terr.zScale = 0.5;
    terr.ySize = 10.0;
    terr.xSize = 10.0;
    terr.xSamples = 80;
    terr.ySamples = 80;
    terr.fractalOctaves = 8;
    terr.fractalLacunarity = 2.0;
    terr.fractalGain = 0.25;
    terr.heightOffset = -(0.5 / 2) - 0.0001;
    height_map_ = world_->addHeightMap(0, 0, terr);
    height_map_->setAppearance("soil1");
}

void RaisimEnvironment::InitializeServer() {
    if (robots_.empty()) {
        throw std::runtime_error("No robot added. Please call AddRobot() first.");
    }
    server_ = std::make_unique<raisim::RaisimServer>(world_.get());
    server_->launchServer(8080);
    // Focus on the first robot's object if available
    tracking_camera_ = false;
    free_camera_ = false;
}

void RaisimEnvironment::AddRobot(const YAML::Node& robot_node, 
                                 const std::string& root_path,
                                 double x_init, double y_init) 
{
    std::string urdf_path;
    try {
        YAML::readParameter(robot_node, "urdf_path", urdf_path);
    }
    catch (std::runtime_error& e) {
        std::cout << "[loma_sim/RaisimWrapper::AddRobot]: " 
                  << "Error reading parameter [" << e.what() << "]" 
                  << std::endl;
    }

    auto articulated_system = world_->addArticulatedSystem(
        root_path + urdf_path);

    int robot_id = static_cast<int>(robots_.size());

    robots_.emplace_back(std::make_unique<Robot>(
        articulated_system, robot_node, robot_id, x_init, y_init));
}

}  // namespace loma_sim