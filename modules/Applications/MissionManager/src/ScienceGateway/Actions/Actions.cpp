/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <memory>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <array>
#include <future>
#include <unistd.h>

#include "MissionManager/ScienceGateway/Actions/Actions.hpp"

namespace crf::applications::missionmanager::sciencegateway {

Actions::Actions(
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable,
    const nlohmann::json& configFile):
        deployable_(deployable),
        initialized_(false),
        logger_("Actions") {
            logger_->debug("CTor");
}

Actions::~Actions() {
    logger_->debug("DTor");
    deinitialize();
}

bool Actions::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return true;  // allow to be initialized many times
    }
    if (!deployable_->initialize()) {
        logger_->error("Cannot initialize the Deployable device");
        return false;
    }
    initialized_ = true;
    return true;
}

bool Actions::deinitialize() {
    logger_->debug("deinitialize Action");
    if (!initialized_) {
        logger_->warn("Already deinitialized");
        return true;  // allow to be deinitialized many times
    }
    if (!deployable_->deinitialize()) {
        logger_->error("Cannot deinitialize the Deployable device");
        return false;
    }
    return true;
}

bool Actions::deploy() {
    logger_->debug("deploy");
    return deployable_->deploy();
}

bool Actions::retract() {
    logger_->debug("retract");
    return deployable_->retract();
}

nlohmann::json Actions::getStatus() {
    logger_->debug("getStatus");
    nlohmann::json json;
    json["initialized"] = initialized_;
    return json;
}

}  // namespace crf::applications::missionmanager::sciencegateway
