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

void RaisimEnvironment::initializeServer() {
    if (robots_.empty()) {
        throw std::runtime_error("No robot added. Please call AddRobot() first.");
    }
    server_ = std::make_unique<raisim::RaisimServer>(world_.get());
    server_->launchServer(8080);
    // Focus on the first robot's object if available
    server_->focusOn(robots_[0]->getArticulatedSystem());
    tracking_camera_ = false;
    free_camera_ = false;
}

int RaisimEnvironment::addRobot(std::shared_ptr<Robot> robot)
{
    robots_.push_back(robot);
    int robot_id = static_cast<int>(robots_.size()) - 1;
    return robot_id;
}


void RaisimEnvironment::updateStates()
{
    for (auto& robot : robots_) {
        robot->updateStates();
    }
}

void RaisimEnvironment::step()
{
    world_->integrate();
}

}  // namespace loma_sim