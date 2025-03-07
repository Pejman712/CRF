/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <fstream>

#include "FTSensorCalibrator/FTSensorCalibratorConfig.hpp"
#include <nlohmann/json.hpp>

using crf::utility::types::JointPositions;

namespace crf {
namespace applications {
namespace ftsensorcalibrator {

FtSensorCalibratorConfig::FtSensorCalibratorConfig():
                    logger_("FtSensorCalibratorConfig"),
                    numberOfJoints_(0),
                    initialJP_(6),
                    endJP_(6),
                    pathResolution_(0),
                    measurementPerPoint_(0),
                    timeBetweenMeasurementsMS_(0),
                    forceThresholdN_(0),
                    torqueThresholdNm_(0),
                    logFile_() {
                        logger_->debug("CTor");
}

bool FtSensorCalibratorConfig::parse(const std::string& filename) {
    std::ifstream sensorCalibData(filename);
    if ((sensorCalibData.rdstate() & std::ifstream::failbit) != 0) {
        logger_->warn("Failed to parse because file could not be opened!");
        return false;
    }
    nlohmann::json calibJSON;
    try {
        sensorCalibData >> calibJSON;

        numberOfJoints_ = calibJSON.at("NumberOfJoints").get<int>();
        for (int i=0; i < 6; i++) {
            initialJP_(i) = calibJSON.at("JointPositions").at("CalibrationStart")[i].get<float>();
            endJP_(i) =
                    calibJSON.at("JointPositions").at("CalibrationEnd")[i].get<float>();
        }

        pathResolution_ = calibJSON.at("PathResolution").get<int>();
        measurementPerPoint_ = calibJSON.at("MeasurementsPerPoint").get<int>();
        timeBetweenMeasurementsMS_ = calibJSON.at("TimeBetweenMeasurementsMS").get<int>();
        forceThresholdN_ = calibJSON.at("ValidationThreshold").at("ForceN").get<float>();
        torqueThresholdNm_ = calibJSON.at("ValidationThreshold").at("TorqueNm").get<float>();

        logFile_ = calibJSON.at("LogFilePath").get<std::string>();
    } catch (const std::exception& e) {
        logger_->warn("Failed to parse because: {}", e.what());
        return false;
    }
    return true;
}

int FtSensorCalibratorConfig::getNumberOfJoints() {
    return numberOfJoints_;
}

utility::types::JointPositions FtSensorCalibratorConfig::getInitialJP() {
    return initialJP_;
}

utility::types::JointPositions FtSensorCalibratorConfig::getEndJP() {
    return endJP_;
}

int FtSensorCalibratorConfig::getPathResolution() {
    return pathResolution_;
}

int FtSensorCalibratorConfig::getMeasurementPerPoint() {
    return measurementPerPoint_;
}

int FtSensorCalibratorConfig::getTimeBetweenMeasurementsMS() {
    return timeBetweenMeasurementsMS_;
}

float FtSensorCalibratorConfig::getForceThresholdN() {
    return forceThresholdN_;
}

float FtSensorCalibratorConfig::getTorqueThresholdNm() {
    return torqueThresholdNm_;
}

std::string FtSensorCalibratorConfig::getLogFile() {
    return logFile_;
}

}  // namespace ftsensorcalibrator
}  // namespace applications
}  // namespace crf
