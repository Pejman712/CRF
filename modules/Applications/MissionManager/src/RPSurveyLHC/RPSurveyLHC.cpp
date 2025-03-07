/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <stdexcept>

#include "MissionManager/RPSurveyLHC/RPSurveyLHC.hpp"

namespace crf::applications::missionmanager::rpsurveylhc {

RPSurveyLHC::RPSurveyLHC(
    std::shared_ptr<crf::actuators::tim::ITIM> tim,
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable,
    std::shared_ptr<crf::sensors::rpsensor::IRPSensor> rpSensor,
    const nlohmann::json& configFile):
        logger_("RPSurveyLHC"),
        actionsPtr_(std::make_shared<Actions>(tim, deployable, rpSensor, configFile)),
        sMachine_(System(actionsPtr_)),
        missionStatus_(),
        pauseTimer_(),
        stopTimer_(),
        setStatus_() {
            logger_->debug("CTor");
}

RPSurveyLHC::~RPSurveyLHC() {
    logger_->debug("DTor");
    stopTimer_ = true;
    if (pauseTimer_.joinable()) {
        pauseTimer_.join();
    }
}

bool RPSurveyLHC::start() {
    logger_->info("start");
    stopTimer_ = true;
    if (pauseTimer_.joinable()) {
        pauseTimer_.join();
    }
    using namespace boost::sml;  // NOLINT
    sMachine_.process_event(Start{});
    if (sMachine_.is("Stop"_s)) {
        logger_->error("Could not start the state machine");
        pause();
        return false;
    }
    return true;
}

bool RPSurveyLHC::next() {
    logger_->info("next");
    stopTimer_ = true;
    if (pauseTimer_.joinable()) {
        pauseTimer_.join();
    }
    using namespace boost::sml;  // NOLINT
    if (sMachine_.is("Initialized"_s)) {
        sMachine_.process_event(Next{});
        if (!sMachine_.is("InStart"_s)) {
            pause();
            return false;
        }
    } else if (sMachine_.is("InStart"_s)) {
        sMachine_.process_event(Next{});
        if (!sMachine_.is("InEnd"_s)) {
            pause();
            return false;
        }
    }
    return true;
}

bool RPSurveyLHC::stop() {
    logger_->info("stop");
    sMachine_.process_event(Stop{});
    using namespace boost::sml;  // NOLINT
    if (!sMachine_.is("Stop"_s)) {
        logger_->error("Could not stop the state machine");
        return false;
    }
    stopTimer_ = true;
    if (pauseTimer_.joinable()) {
        pauseTimer_.join();
    }
    return true;
}

bool RPSurveyLHC::pause() {
    logger_->info("pause");
    sMachine_.process_event(Pause{});
    stopTimer_ = false;
    if (!pauseTimer_.joinable()) {
        pauseTimer_ = std::thread(&RPSurveyLHC::timer, this);
    }
    return true;
}

bool RPSurveyLHC::resume() {
    logger_->info("resume");
    stopTimer_ = true;
    if (pauseTimer_.joinable()) {
        pauseTimer_.join();
    }
    sMachine_.process_event(Resume{});
    return true;
}

bool RPSurveyLHC::goHome() {
    logger_->info("goHome");
    sMachine_.process_event(GoHome{});
    return true;
}

bool RPSurveyLHC::recharge() {
    logger_->info("recharge");
    sMachine_.process_event(Recharge{});
    return true;
}

nlohmann::json RPSurveyLHC::getStatus() {
    using namespace boost::sml;  // NOLINT
    nlohmann::json stat = actionsPtr_->getStatus();
    if (sMachine_.is("Stop"_s)) {
        missionStatus_ = MissionStatus::Stopped;
    } else if (!stopTimer_) {
        missionStatus_ = MissionStatus::Paused;
    } else if (!sMachine_.is("Stop"_s)) {
        missionStatus_ = MissionStatus::Started;
    } else {
        missionStatus_ = MissionStatus::NotDefined;
    }
    stat["status"] = missionStatus_;
    return stat;
}

bool RPSurveyLHC::setStatus(const nlohmann::json& json) {
    logger_->info("setStatus");
    RPSurveyLHC::clearGuards();
    if (json.contains("retractArm")) {
        if (json.at("retractArm").get<bool>()) {
            logger_->info("setStatus in retractArm");
            setStatus_.Retract = true;
        }
    }
    if (json.contains("deployArm")) {
        if (json.at("deployArm").get<bool>()) {
            logger_->info("setStatus in deployArm");
            setStatus_.Deploy = true;
        }
    }
    if (json.contains("surveyData")) {
        actionsPtr_->setSurveyParameters(json.at("surveyData"));
        return true;
    }
    sMachine_.process_event(setStatus_);
    RPSurveyLHC::clearGuards();
    sMachine_.process_event(Reset{});
    return true;
}

bool RPSurveyLHC::emergency() {
    logger_->info("emergency");
    sMachine_.process_event(Emergency{});
    return true;
}

bool RPSurveyLHC::rearm() {
    logger_->info("rearm");
    sMachine_.process_event(Rearm{});
    return true;
}

// Private methods

void RPSurveyLHC::clearGuards() {
    setStatus_.Deploy = false;
    setStatus_.Retract = false;
}

void RPSurveyLHC::timer() {
    logger_->info("Timer started");
    auto startTime = std::chrono::high_resolution_clock::now();
    while (!stopTimer_) {
        std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - startTime);
        if (elapsed > std::chrono::seconds(1800)) {  // 30 minutes
            RPSurveyLHC::stop();
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        logger_->warn("State machine paused");
    }
    logger_->info("Timer stopped");
}

}  // namespace crf::applications::missionmanager::rpsurveylhc
