/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <stdexcept>

#include "MissionManager/ScienceGateway/ScienceGateway.hpp"

namespace crf::applications::missionmanager::sciencegateway {

ScienceGateway::ScienceGateway(
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable,
    const nlohmann::json& configFile):
        actionsPtr_(std::make_shared<Actions>(deployable, configFile)),
        sMachine_(
            System(actionsPtr_),
            EasyMission(actionsPtr_),
            MediumMission(actionsPtr_),
            HardMission(actionsPtr_)),
        missionStatus_(),
        logger_("RPSurveyLHC") {
            logger_->debug("CTor");
}

ScienceGateway::~ScienceGateway() {
    logger_->debug("DTor");
}

bool ScienceGateway::start() {
    logger_->debug("start");
    using namespace boost::sml;  // NOLINT
    if (!sMachine_.is("Stop"_s)) {
        logger_->warn("Already started");
        return true;
    }
    sMachine_.process_event(Start{});
    if (!sMachine_.is("Initialized"_s)) {
        logger_->error("Could not start the state machine");
        return false;
    }
    logger_->info("Successfully start");
    return true;
}

bool ScienceGateway::stop() {
    using namespace boost::sml;  // NOLINT
    logger_->debug("stop");
    if (sMachine_.is("Stop"_s)) {
        logger_->warn("Already stopped");
        return true;
    }
    sMachine_.process_event(Stop{});
    if (!sMachine_.is("Stop"_s)) {
        logger_->error("Could not stop the state machine");
        return false;
    }
    logger_->info("Successfully stop");
    return true;
}

bool ScienceGateway::goHome() {
    using namespace boost::sml;  // NOLINT
    logger_->debug("goHome");
    if (sMachine_.is("Stop"_s)) {
        logger_->error("Not Initialized");
        return false;
    }
    sMachine_.process_event(GoHome{});
    if (!sMachine_.is("Initialized"_s)) {
        logger_->error("Retract failed");
        return false;
    }
    logger_->info("Successfully goHome");
    return true;
}

bool ScienceGateway::next() {
    using namespace boost::sml;  // NOLINT
    logger_->debug("next");
    if (sMachine_.is(state<EasyMission>)) {
        logger_->info("Easy mission");
        sMachine_.process_event(Next{});
        if (!sMachine_.is<decltype(state<EasyMission>)>("Deployed"_s)) {
            logger_->error("Deploy failed in easy mission");
            return false;
        }
    } else if (sMachine_.is(state<MediumMission>)) {
        logger_->info("Medium mission");
        sMachine_.process_event(Next{});
        if (sMachine_.is<decltype(state<MediumMission>)>("Deployed"_s)) {
            logger_->error("Deploy failed in medium mission");
            return false;
        }
    } else if (sMachine_.is(state<HardMission>)) {
        logger_->info("Hard mode");
        sMachine_.process_event(Next{});
        if (sMachine_.is<decltype(state<HardMission>)>("Deployed"_s)) {
            logger_->error("Deploy failed in hard mission");
            return false;
        }
    } else {
        logger_->error("Unknown state to do next command");
        return false;
    }
    return true;
}

nlohmann::json ScienceGateway::getStatus() {
    logger_->debug("getStatus");
    using namespace boost::sml;  // NOLINT
    nlohmann::json stat = actionsPtr_->getStatus();
    if (sMachine_.is("Stop"_s)) {
        missionStatus_ = MissionStatus::Stopped;
    } else if (!sMachine_.is("Stop"_s)) {
        missionStatus_ = MissionStatus::Started;
    } else {
        missionStatus_ = MissionStatus::NotDefined;
    }
    stat["status"] = missionStatus_;
    return stat;
}

bool ScienceGateway::setStatus(const nlohmann::json& json) {
    logger_->debug("setStatus");
    using namespace boost::sml;  // NOLINT
    ScienceGateway::clearGuards();
    try {
        if (!sMachine_.is("Initialized"_s)) {
            logger_->error("The state machine is not Initialized");
            return false;
        }
        if (json.contains("MissionSelect")) {
            setMission_.mission = json.at("MissionSelect").get<MissionSelect>();
        } else {
            logger_->error("Information doesn't exsit in json file.");
            return false;
        }
    } catch (const std::exception& e) {
        logger_->error("Set status incorrect! => {}", e.what());
        return false;
    }
    sMachine_.process_event(setMission_);
    return true;
}

bool ScienceGateway::pause() {return false;}
bool ScienceGateway::resume() {return false;}
bool ScienceGateway::recharge() {return false;}
bool ScienceGateway::emergency() {return false;}
bool ScienceGateway::rearm() {return false;}

// Private methods
void ScienceGateway::clearGuards() {
    setMission_.mission = MissionSelect::NotSet;
}

}  // namespace crf::applications::missionmanager::sciencegateway
