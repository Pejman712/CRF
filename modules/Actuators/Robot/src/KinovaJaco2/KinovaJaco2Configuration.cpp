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

#include "Robot/KinovaJaco2/KinovaJaco2Configuration.hpp"

namespace crf::actuators::robot {

KinovaJaco2Configuration::KinovaJaco2Configuration(const nlohmann::json& robotConfig) :
    RobotConfiguration(robotConfig),
    logger_("KinovaJaco2Configuration") {
        logger_->debug("CTor");
        parse(robotConfig);
}

std::string KinovaJaco2Configuration::getSerialNumber() const {
    return serialNumber_;
}

KinovaJaco2NetworkConfiguration KinovaJaco2Configuration::getNetworkConfiguration() const {
    return networkConfiguration_;
}

// Private

void KinovaJaco2Configuration::parse(const nlohmann::json& robotJSON) {
    logger_->debug("parse");
    try {
        serialNumber_ =  robotJSON.at("SerialNumber").get<std::string>();
        nlohmann::json ethComm = robotJSON["EthernetCommunication"];
        networkConfiguration_.localAddressIP = ethComm["LocalPcIPAddress"].get<std::string>();
        networkConfiguration_.robotAddressIP = ethComm["IPAdress"].get<std::string>();
        networkConfiguration_.subnetMask = ethComm["SubnetMask"].get<std::string>();
        networkConfiguration_.port = ethComm["Port"].get<unsigned int>();
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        throw std::invalid_argument(
            "The configuration file provided could not be parsed in KinovaJaco2Configuration");
    }
}

}  // namespace crf::actuators::robot
