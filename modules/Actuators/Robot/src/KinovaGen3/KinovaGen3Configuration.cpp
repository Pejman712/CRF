/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "Robot/KinovaGen3/KinovaGen3Configuration.hpp"

namespace crf::actuators::robot {

KinovaGen3Configuration::KinovaGen3Configuration(const nlohmann::json& robotJSON) :
    RobotConfiguration(robotJSON),
    logger_("KinovaGen3Configuration") {
        logger_->debug("CTor");
        parse(robotJSON);
}

std::string KinovaGen3Configuration::getUsername() const {
    return username_;
}

std::string KinovaGen3Configuration::getPassword() const {
    return password_;
}


std::string KinovaGen3Configuration::getIPAddress() const {
    return ipAddress_;
}

uint32_t KinovaGen3Configuration::getTCPPort() const {
    return tcpPort_;
}

uint32_t KinovaGen3Configuration::getUDPPort() const {
    return udpPort_;
}

uint32_t KinovaGen3Configuration::getSessionTimeout() const {
    return sessionTimeout_;
}

uint32_t KinovaGen3Configuration::getConnectionTimeout() const {
    return connectionTimeout_;
}

// Private

void KinovaGen3Configuration::parse(const nlohmann::json& robotConfig) {
    logger_->debug("parse");
    try {
        username_ = robotConfig.at("Username").get<std::string>();
        password_ = robotConfig.at("Password").get<std::string>();
        ipAddress_ = robotConfig.at("IPAddress").get<std::string>();
        tcpPort_ = robotConfig.at("TCPPort").get<uint32_t>();
        udpPort_ = robotConfig.at("UDPPort").get<uint32_t>();
        sessionTimeout_ = robotConfig.at("SessionTimeout").get<uint32_t>();
        connectionTimeout_ = robotConfig.at("ConnectionTimeout").get<uint32_t>();
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        throw std::invalid_argument(
            "The configuration file provided could not be parsed in KinovaGen3Configuration");
    }
}

}  // namespace crf::actuators::robot
