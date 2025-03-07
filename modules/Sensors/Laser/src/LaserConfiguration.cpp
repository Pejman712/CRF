/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <exception>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "Laser/LaserConfiguration.hpp"

namespace crf {
namespace sensors {
namespace laser {

LaserConfiguration::LaserConfiguration():
    logger_("LaserConfiguration"),
    laserParameters_{},
    framelimits_(),
    angularResolution_(0.0f),
    scanInterval_(0) {}

bool LaserConfiguration::parse(const nlohmann::json& laserJSON) {
    logger_->info("Parsing laser configuration file");
    cleanUp();
    try {
        laserParameters_.minRange = laserJSON.at("Range").at("Minimum").get<float>();
        laserParameters_.maxRange = laserJSON.at("Range").at("Maximum").get<float>();
        angularResolution_ = laserJSON.at("AngularResolution").get<float>();
        laserParameters_.minAngle = laserJSON.at("Angle").at("Minimum").get<float>();
        laserParameters_.maxAngle = laserJSON.at("Angle").at("Maximum").get<float>();
        laserParameters_.scanSize = laserJSON.at("ScanSize").get<unsigned int>();
        laserParameters_.frequency = laserJSON.at("Frequency").get<unsigned int>();
        laserParameters_.LidarVendorID = laserJSON.at("LiDARIDs").at("LidarVendorID").get<unsigned int>();
        laserParameters_.LidarProductID = laserJSON.at("LiDARIDs").at("LidarProductID").get<unsigned int>();
        framelimits_.x_min = laserJSON.at("FrameLimits").at("X_Minimum").get<float>();
        framelimits_.x_max = laserJSON.at("FrameLimits").at("X_Maximum").get<float>();
        framelimits_.y_min = laserJSON.at("FrameLimits").at("Y_Minimum").get<float>();
        framelimits_.y_max = laserJSON.at("FrameLimits").at("Y_Maximum").get<float>();
        framelimits_.z_min = laserJSON.at("FrameLimits").at("Z_Minimum").get<float>();
        framelimits_.z_max = laserJSON.at("FrameLimits").at("Z_Maximum").get<float>();
        auto interval = std::chrono::microseconds(
                laserJSON.at("ScanIntervalUs").get<unsigned int>());
        if (!setScanIntervalUS(interval)) {
            cleanUp();
            return false;
        }
    } catch (const std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        cleanUp();
        return false;
    }
    return true;
}

void LaserConfiguration::cleanUp() {
    laserParameters_ = {0, 0, 0, 0, 0};
    framelimits_ = {0, 0, 0, 0, 0, 0};
    angularResolution_ = 0;
    scanInterval_ = std::chrono::microseconds(0);
}

LaserParameters LaserConfiguration::getLaserParameters() {
    return laserParameters_;
}

FrameLimits LaserConfiguration::getFrameLimits() {
    return framelimits_;
}

bool LaserConfiguration::setLaserParameters(const LaserParameters& laserParameters) {
    if (laserParameters.scanSize == 0) {
        logger_->warn("Invalid scan size");
        return false;
    }
    laserParameters_ = laserParameters;
    return true;
}

std::chrono::microseconds LaserConfiguration::getScanIntervalUS() {
    return scanInterval_;
}

bool LaserConfiguration::setScanIntervalUS(const std::chrono::microseconds& interval) {
    if (interval.count() == 0) {
        logger_->warn("Invalid scan interval {}", interval.count());
        return false;
    }
    scanInterval_ = interval;
    return true;
}

float LaserConfiguration::getAngularResolution() {
    return angularResolution_;
}

}  // namespace laser
}  // namespace sensors
}  // namespace crf
