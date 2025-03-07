/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>

#include <nlohmann/json.hpp>

// State Spaces
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "PathPlanner/OMPLGeometricPlanner/OMPLStateSpaceConfiguration.hpp"

using ompl::base::CompoundStateSpace;
using ompl::base::RealVectorStateSpace;
using ompl::base::RealVectorBounds;
using ompl::base::SO2StateSpace;
using ompl::base::SO3StateSpace;
using ompl::base::SE2StateSpace;
using ompl::base::SE3StateSpace;

namespace crf::navigation::pathplanner {

OMPLStateSpaceConfiguration::OMPLStateSpaceConfiguration(const nlohmann::json& config) {
    crf::utility::logger::EventLogger logger("OMPLStateSpaceConfiguration");
    for (uint64_t i = 0; i < config.size(); i++) {
        if (config[i]["Type"] == "Real") {
            auto realSpace = std::make_shared<RealVectorStateSpace>(
                config[i]["Dimension"].get<uint64_t>());

            if (config[i].contains("Bounds")) {
                realSpace->setBounds(
                    config[i]["Bounds"]["Min"].get<double>(),
                    config[i]["Bounds"]["Max"].get<double>());
            }

            CompoundStateSpace::addSubspace(
                realSpace,
                config[i]["Weight"].get<double>());
            continue;
        }
        if (config[i]["Type"] == "SO2") {
            CompoundStateSpace::addSubspace(
                std::make_shared<SO2StateSpace>(),
                config[i]["Weight"].get<double>());
            continue;
        }
        if (config[i]["Type"] == "SO3") {
            CompoundStateSpace::addSubspace(
                std::make_shared<SO3StateSpace>(),
                config[i]["Weight"].get<double>());
            continue;
        }
        if (config[i]["Type"] == "SE2") {
            auto se2Space = std::make_shared<SE2StateSpace>();

            if (config[i].contains("Bounds")) {
                RealVectorBounds bounds(config[i]["Dimension"].get<uint64_t>());
                bounds.setLow(config[i]["Bounds"]["Min"].get<double>());
                bounds.setHigh(config[i]["Bounds"]["Max"].get<double>());
                se2Space->setBounds(bounds);
            }

            CompoundStateSpace::addSubspace(
                se2Space,
                config[i]["Weight"].get<double>());
            continue;
        }
        if (config[i]["Type"] == "SE3") {
            auto se3Space = std::make_shared<SE3StateSpace>();

            if (config[i].contains("Bounds")) {
                RealVectorBounds bounds(config[i]["Dimension"].get<uint64_t>());
                bounds.setLow(config[i]["Bounds"]["Min"].get<double>());
                bounds.setHigh(config[i]["Bounds"]["Max"].get<double>());
                se3Space->setBounds(bounds);
            }

            CompoundStateSpace::addSubspace(
                se3Space,
                config[i]["Weight"].get<double>());
            continue;
        }
        throw std::invalid_argument("The requested state space is not available!");
    }
    std::ostringstream oss;
    CompoundStateSpace::printSettings(oss);
    logger->info("State space settings:\n{}", oss.str());
    CompoundStateSpace::sanityChecks();
    CompoundStateSpace::lock();
}

}  // namespace crf::navigation::pathplanner
