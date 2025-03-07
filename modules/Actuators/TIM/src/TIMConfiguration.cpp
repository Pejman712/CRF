/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <regex>
#include <string>
#include <chrono>
#include <map>
#include <utility>

#include <nlohmann/json.hpp>

#include "TIM/TIMConfiguration.hpp"

namespace crf::actuators::tim {

TIMConfiguration::TIMConfiguration() :
    logger_("TIMConfiguration"),
    ipAddress_(""),
    rack_(0),
    slot_(0),
    maxHeartbeatValue_(),
    updateInterval_(),
    commandTimeout_(),
    positionOffset_(),
    obstacleDBLocation_(),
    obstacleVariablesLocation_(),
    alarmDBLocation_(),
    commandsDBLocation_(),
    statusDBLocation_(),
    alarmVariablesLocation_(),
    commandsVariablesLocation_(),
    statusVariablesLocation_() {
    logger_->debug("CTor");
}

bool TIMConfiguration::parse(const nlohmann::json& timJSON) {
    cleanup();
    try {
        ipAddress_ = timJSON["PLC"]["IP"].get<std::string>();
        if (!isValidIPAddress(ipAddress_)) {
            logger_->error("Not valid IP address");
            cleanup();
            return false;
        }
        rack_ = timJSON["PLC"]["Rack"].get<unsigned int>();
        slot_ = timJSON["PLC"]["Slot"].get<unsigned int>();
        maxHeartbeatValue_ = timJSON["PLC"]["maxHeartbeatValue"].get<int>();
        updateInterval_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::milliseconds(timJSON["PLC"]["UpdateIntervalInMilliseconds"].get<int>()));
        commandTimeout_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::seconds(timJSON["PLC"]["commandWaitingTimeInSeconds"].get<int>()));
        positionOffset_ = timJSON["PositionOffset"].get<float>();
        limits.maximumPosition = timJSON["Limits"]["MaximumPosition"].get<float>();
        limits.minimumPosition = timJSON["Limits"]["MinimumPosition"].get<float>();
        limits.maximumVelocity = timJSON["Limits"]["MaximumVelocity"].get<float>();

        nlohmann::json obstaclesJSON = timJSON["Datablocks"]["Obstacles"];
        obstacleDBLocation_ = {obstaclesJSON["DatablockNumber"].get<unsigned int>(),
            obstaclesJSON["DatablockLength"].get<unsigned int>(),
            obstaclesJSON["ObstacleLength"].get<unsigned int>()};
        for (auto& [key, value] : obstaclesJSON["Variables"].items()) {
            obstacleVariablesLocation_.insert({key, {value["RegisterOffset"].get<unsigned int>(),
                value["BitNumber"].get<unsigned int>()}});
        }

        nlohmann::json alarmsJSON = timJSON["Datablocks"]["TIMAlarms"];
        alarmDBLocation_ = {alarmsJSON["DatablockNumber"].get<unsigned int>(),
            alarmsJSON["DatablockLength"].get<unsigned int>()};
        for (auto& [key, value] : alarmsJSON["Variables"].items()) {
            alarmVariablesLocation_.insert({key, {value["RegisterOffset"].get<unsigned int>(),
                value["BitNumber"].get<unsigned int>()}});
        }

        nlohmann::json commandsJSON = timJSON["Datablocks"]["TIMCommands"];
        commandsDBLocation_ = {commandsJSON["DatablockNumber"].get<unsigned int>(),
            commandsJSON["DatablockLength"].get<unsigned int>()};
        for (auto& [key, value] : commandsJSON["Variables"].items()) {
            commandsVariablesLocation_.insert({key, {value["RegisterOffset"].get<unsigned int>(),
                value["BitNumber"].get<unsigned int>()}});
        }

        nlohmann::json settingsJSON = timJSON["Datablocks"]["TIMSettings"];
        settingsDBLocation_ = {settingsJSON["DatablockNumber"].get<unsigned int>(),
            settingsJSON["DatablockLength"].get<unsigned int>()};
        for (auto& [key, value] : settingsJSON["Variables"].items()) {
            settingsVariablesLocation_.insert({key, {value["RegisterOffset"].get<unsigned int>(),
                value["BitNumber"].get<unsigned int>()}});
        }

        nlohmann::json statusJSON = timJSON["Datablocks"]["TIMStatus"];
        statusDBLocation_ = {statusJSON["DatablockNumber"].get<unsigned int>(),
            statusJSON["DatablockLength"].get<unsigned int>()};
        for (auto& [key, value] : statusJSON["Variables"].items()) {
            statusVariablesLocation_.insert({key, {value["RegisterOffset"].get<unsigned int>(),
                value["BitNumber"].get<unsigned int>()}});
        }
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        cleanup();
        return false;
    }
    return true;
}

std::string TIMConfiguration::getIPAddress() {
    return ipAddress_;
}

unsigned int TIMConfiguration::getRack() {
    return rack_;
}

unsigned int TIMConfiguration::getSlot() {
    return slot_;
}

int TIMConfiguration::getMaxHeartbeatValue() {
    return maxHeartbeatValue_;
}

std::chrono::microseconds TIMConfiguration::getUpdateInterval() {
    return updateInterval_;
}

std::chrono::microseconds TIMConfiguration::getCommandTimeout() {
    return commandTimeout_;
}

float TIMConfiguration::getPositionOffset() {
    return positionOffset_;
}

crf::actuators::tim::TIMLimits TIMConfiguration::getTIMLimits() {
    return limits;
}

std::array<unsigned int, 3> TIMConfiguration::getObstacleDBLocation() {
    return obstacleDBLocation_;
}

std::map<std::string, std::array<unsigned int, 2>> TIMConfiguration::getObstacleVariablesLocation() {  // NOLINT
    return obstacleVariablesLocation_;
}

std::array<unsigned int, 2> TIMConfiguration::getAlarmDBLocation() {
    return alarmDBLocation_;
}

std::array<unsigned int, 2> TIMConfiguration::getCommandsDBLocation() {
    return commandsDBLocation_;
}

std::array<unsigned int, 2> TIMConfiguration::getSettingsDBLocation() {
    return settingsDBLocation_;
}

std::array<unsigned int, 2> TIMConfiguration::getStatusDBLocation() {
    return statusDBLocation_;
}

std::map<std::string, std::array<unsigned int, 2>> TIMConfiguration::getAlarmVariablesLocation() {  // NOLINT
    return alarmVariablesLocation_;
}

std::map<std::string, std::array<unsigned int, 2>> TIMConfiguration::getCommandsVariablesLocation() {  // NOLINT
    return commandsVariablesLocation_;
}

std::map<std::string, std::array<unsigned int, 2>> TIMConfiguration::getSettingsVariablesLocation() {  // NOLINT
    return settingsVariablesLocation_;
}

std::map<std::string, std::array<unsigned int, 2>> TIMConfiguration::getStatusVariablesLocation() {  // NOLINT
    return statusVariablesLocation_;
}

void TIMConfiguration::cleanup() {
    ipAddress_ = "";
    rack_ = 0;
    slot_ = 0;
    limits.maximumPosition = 0;
    limits.minimumPosition = 0;
    limits.maximumVelocity = 0;
    obstacleDBLocation_ = {0, 0};
    obstacleVariablesLocation_.clear();
    alarmDBLocation_ = {0, 0};
    commandsDBLocation_ = {0, 0};
    settingsDBLocation_ = {0, 0};
    statusDBLocation_ = {0, 0};
    alarmVariablesLocation_.clear();
    commandsVariablesLocation_.clear();
    settingsVariablesLocation_.clear();
    statusVariablesLocation_.clear();
}

bool TIMConfiguration::isValidIPAddress(const std::string& ip) {
    const std::regex pattern("(\\d{1,3}(\\.\\d{1,3}){3})");
    return std::regex_match(ip, pattern);
}

}  // namespace crf::actuators::tim
