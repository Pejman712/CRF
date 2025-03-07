/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "KinovaArm/KinovaArmConfiguration.hpp"

namespace crf::actuators::kinovaarm {

KinovaArmConfiguration::KinovaArmConfiguration() :
    RobotArmConfiguration(),
    serialNumber_(),
    networkConfiguration_() {
}

bool KinovaArmConfiguration::parse(const nlohmann::json& robotJSON) {
    logger_->debug("parse");
    cleanup();
    if (!RobotArmConfiguration::parse(robotJSON)) {
        return false;
    }
    try {
        serialNumber_ =  robotJSON.at("SerialNumber").get<std::string>();
        nlohmann::json ethComm = robotJSON["EthernetCommunication"];
        networkConfiguration_.localAddressIP = ethComm["LocalPcIPAddress"].get<std::string>();
        networkConfiguration_.robotAddressIP = ethComm["IPAdress"].get<std::string>();
        networkConfiguration_.subnetMask = ethComm["SubnetMask"].get<std::string>();
        networkConfiguration_.port = ethComm["Port"].get<unsigned int>();
    } catch (std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        cleanup();
        return false;
    }
    return true;
}

std::string KinovaArmConfiguration::getSerialNumber() {
    return serialNumber_;
}

KinovaJacoNetworkConfiguration KinovaArmConfiguration::getNetworkConfiguration() {
    return networkConfiguration_;
}

void KinovaArmConfiguration::cleanup() {
    RobotArmConfiguration::cleanup();
    serialNumber_ = "";
    networkConfiguration_ = KinovaJacoNetworkConfiguration();
}

}  // namespace crf::actuators::kinovaarm
