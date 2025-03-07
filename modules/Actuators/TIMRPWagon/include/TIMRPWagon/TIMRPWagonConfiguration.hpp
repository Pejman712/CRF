/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <chrono>
#include <map>
#include <utility>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"

namespace crf::actuators::timrpwagon {

class TIMRPWagonConfiguration {
 public:
    TIMRPWagonConfiguration();
    virtual ~TIMRPWagonConfiguration() = default;

    virtual bool parse(const nlohmann::json& timJSON);
    bool parse(const std::string&) = delete;

    std::string getIPAddress();
    unsigned int getRack();
    unsigned int getSlot();
    std::chrono::microseconds getUpdateInterval();
    std::chrono::microseconds getCommandTimeout();

    std::pair<unsigned int, unsigned int> getCommandsDatablockLocation();
    std::pair<unsigned int, unsigned int> getStatusDatablockLocation();
    std::map<std::string, std::pair<unsigned int, unsigned int>> getCommandsVariablesDBLocation();
    std::map<std::string, std::pair<unsigned int, unsigned int>> getStatusVariablesDBLocation();

 private:
    utility::logger::EventLogger logger_;
    std::string ipAddress_;
    unsigned int rack_;
    unsigned int slot_;
    std::chrono::microseconds updateInterval_;
    std::chrono::microseconds commandTimeout_;

    std::pair<unsigned int, unsigned int> commandsDBLocation_;
    std::pair<unsigned int, unsigned int> statusDBLocation_;
    std::map<std::string, std::pair<unsigned int, unsigned int>> commandsVariablesDBLocation_;
    std::map<std::string, std::pair<unsigned int, unsigned int>> statusVariablesDBLocation_;

    void cleanup();
    bool isValidIPAddress(const std::string& ip);
};

}  // namespace crf::actuators::timrpwagon
