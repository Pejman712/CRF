/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <regex>
#include <string>
#include <chrono>
#include <map>
#include <utility>

#include <nlohmann/json.hpp>

#include "TIMRPWagon/TIMRPWagonConfiguration.hpp"

namespace crf::actuators::timrpwagon {

TIMRPWagonConfiguration::TIMRPWagonConfiguration() :
    logger_("TIMRPWagonConfiguration"),
    ipAddress_(""),
    rack_(0),
    slot_(0) {
    logger_->debug("CTor");
}

bool TIMRPWagonConfiguration::parse(const nlohmann::json& timJSON) {
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
        updateInterval_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::seconds(timJSON["PLC"]["UpdateIntervalInSeconds"].get<int>()));
        commandTimeout_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::seconds(timJSON["PLC"]["commandWaitingTimeInSeconds"].get<int>()));

        nlohmann::json commandsJSON = timJSON["Datablocks"]["Commands"];
        commandsDBLocation_ = std::make_pair(commandsJSON["DatablockNumber"].get<unsigned int>(),
            commandsJSON["DatablockLength"].get<unsigned int>());
        for (auto& [key, value] : commandsJSON["Variables"].items()) {
            commandsVariablesDBLocation_.insert({key, std::make_pair(
                value["RegisterOffset"].get<unsigned int>(),
                value["BitNumber"].get<unsigned int>())});
        }

        nlohmann::json statusJSON = timJSON["Datablocks"]["Status"];
        statusDBLocation_ = std::make_pair(statusJSON["DatablockNumber"].get<unsigned int>(),
            statusJSON["DatablockLength"].get<unsigned int>());
        for (auto& [key, value] : statusJSON["Variables"].items()) {
            statusVariablesDBLocation_.insert({key, std::make_pair(
                value["RegisterOffset"].get<unsigned int>(),
                value["BitNumber"].get<unsigned int>())});
        }
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        cleanup();
        return false;
    }
    return true;
}

std::string TIMRPWagonConfiguration::getIPAddress() {
    return ipAddress_;
}

unsigned int TIMRPWagonConfiguration::getRack() {
    return rack_;
}

unsigned int TIMRPWagonConfiguration::getSlot() {
    return slot_;
}

std::chrono::microseconds TIMRPWagonConfiguration::getUpdateInterval() {
    return updateInterval_;
}

std::chrono::microseconds TIMRPWagonConfiguration::getCommandTimeout() {
    return commandTimeout_;
}

std::pair<unsigned int, unsigned int> TIMRPWagonConfiguration::getCommandsDatablockLocation() {
    return commandsDBLocation_;
}

std::pair<unsigned int, unsigned int> TIMRPWagonConfiguration::getStatusDatablockLocation() {
    return statusDBLocation_;
}

std::map<std::string, std::pair<unsigned int, unsigned int>> TIMRPWagonConfiguration::getCommandsVariablesDBLocation() {  // NOLINT
    return commandsVariablesDBLocation_;
}

std::map<std::string, std::pair<unsigned int, unsigned int>> TIMRPWagonConfiguration::getStatusVariablesDBLocation() {  // NOLINT
    return statusVariablesDBLocation_;
}

void TIMRPWagonConfiguration::cleanup() {
    ipAddress_ = "";
    rack_ = 0;
    slot_ = 0;
    commandsDBLocation_ = std::make_pair(0, 0);
    statusDBLocation_ = std::make_pair(0, 0);
    commandsVariablesDBLocation_.clear();
    statusVariablesDBLocation_.clear();
}

bool TIMRPWagonConfiguration::isValidIPAddress(const std::string& ip) {
    const std::regex pattern("(\\d{1,3}(\\.\\d{1,3}){3})");
    return std::regex_match(ip, pattern);
}

}  // namespace crf::actuators::timrpwagon
