/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero & Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>

#include <nlohmann/json.hpp>

#include "Laser/VelodyneHDL/VelodyneHDLLaserConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace laser {

VelodyneHDLLaserConfiguration::VelodyneHDLLaserConfiguration() :
    LaserConfiguration(),
    correctionParametersFilePath_(),
    networkConfiguration_() {
    logger_->debug("CTor");
}

bool VelodyneHDLLaserConfiguration::parse(const nlohmann::json& laserJSON) {
    logger_->debug("parse");
    cleanUp();
    if (!LaserConfiguration::parse(laserJSON)) {
        return false;
    }
    try {
        networkConfiguration_.ipAddress =  laserJSON.at("IPAddress").get<std::string>();
        networkConfiguration_.udpPort =  laserJSON.at("UPDPort").get<uint16_t>();
        correctionParametersFilePath_ += laserJSON.at("CorrectionFile").get<std::string>();
        if (!correctionParametersFilePath_.empty()) {
            std::string systemPath = __FILE__;
            systemPath = systemPath.substr(0, systemPath.find("cpproboticframework"));
            correctionParametersFilePath_ = systemPath + correctionParametersFilePath_;
        }
    } catch (std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        cleanUp();
        return false;
    }
    return true;
}

VelodyneHDLLaserNetworkConfiguration VelodyneHDLLaserConfiguration::getNetworkConfiguration() {
    logger_->debug("getNetworkConfiguration");
    return networkConfiguration_;
}

std::string VelodyneHDLLaserConfiguration::getCorrectionParametersFilePath() {
    logger_->debug("getCorrectionParametersFilePath");
    return correctionParametersFilePath_;
}

void VelodyneHDLLaserConfiguration::cleanUp() {
    logger_->debug("getCorrectionParametersFilePath");
    LaserConfiguration::cleanUp();
    networkConfiguration_ = VelodyneHDLLaserNetworkConfiguration();
    correctionParametersFilePath_ = "";
}

}  // namespace laser
}  // namespace sensors
}  // namespace crf
