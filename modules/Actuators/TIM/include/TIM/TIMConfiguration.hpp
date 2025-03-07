/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
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

namespace crf::actuators::tim {

struct TIMLimits {
    float maximumPosition = 0;
    float minimumPosition = 0;
    float maximumVelocity = 0;
};

class TIMConfiguration {
 public:
    TIMConfiguration();
    virtual ~TIMConfiguration() = default;

    virtual bool parse(const nlohmann::json& timJSON);
    bool parse(const std::string&) = delete;

    std::string getIPAddress();
    unsigned int getRack();
    unsigned int getSlot();
    int getMaxHeartbeatValue();
    std::chrono::microseconds getUpdateInterval();
    std::chrono::microseconds getCommandTimeout();
    float getPositionOffset();
    crf::actuators::tim::TIMLimits getTIMLimits();

    std::array<unsigned int, 3> getObstacleDBLocation();
    std::map<std::string, std::array<unsigned int, 2>> getObstacleVariablesLocation();

    std::array<unsigned int, 2> getAlarmDBLocation();
    std::array<unsigned int, 2> getCommandsDBLocation();
    std::array<unsigned int, 2> getSettingsDBLocation();
    std::array<unsigned int, 2> getStatusDBLocation();
    std::map<std::string, std::array<unsigned int, 2>> getAlarmVariablesLocation();
    std::map<std::string, std::array<unsigned int, 2>> getCommandsVariablesLocation();
    std::map<std::string, std::array<unsigned int, 2>> getSettingsVariablesLocation();
    std::map<std::string, std::array<unsigned int, 2>> getStatusVariablesLocation();

 private:
    utility::logger::EventLogger logger_;
    std::string ipAddress_;
    unsigned int rack_;
    unsigned int slot_;
    int maxHeartbeatValue_;
    std::chrono::microseconds updateInterval_;
    std::chrono::microseconds commandTimeout_;
    float positionOffset_;
    TIMLimits limits;

    std::array<unsigned int, 3> obstacleDBLocation_;
    std::map<std::string, std::array<unsigned int, 2>> obstacleVariablesLocation_;

    std::array<unsigned int, 2> alarmDBLocation_;
    std::array<unsigned int, 2> commandsDBLocation_;
    std::array<unsigned int, 2> settingsDBLocation_;
    std::array<unsigned int, 2> statusDBLocation_;
    std::map<std::string, std::array<unsigned int, 2>> alarmVariablesLocation_;
    std::map<std::string, std::array<unsigned int, 2>> commandsVariablesLocation_;
    std::map<std::string, std::array<unsigned int, 2>> settingsVariablesLocation_;
    std::map<std::string, std::array<unsigned int, 2>> statusVariablesLocation_;

    void cleanup();
    bool isValidIPAddress(const std::string& ip);
};

}  // namespace crf::actuators::tim
