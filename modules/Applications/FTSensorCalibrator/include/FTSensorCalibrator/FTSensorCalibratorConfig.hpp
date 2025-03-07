#pragma once
/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

namespace crf {
namespace applications {
namespace ftsensorcalibrator {

class FtSensorCalibratorConfig {
 public:
    FtSensorCalibratorConfig();
    ~FtSensorCalibratorConfig() = default;
    bool parse(const std::string& filename);
    int getNumberOfJoints();
    utility::types::JointPositions getInitialJP();
    utility::types::JointPositions getEndJP();
    int getPathResolution();
    int getMeasurementPerPoint();
    int getTimeBetweenMeasurementsMS();
    float getForceThresholdN();
    float getTorqueThresholdNm();
    std::string getLogFile();

 private:
     utility::logger::EventLogger logger_;
     int numberOfJoints_;
     utility::types::JointPositions initialJP_;
     utility::types::JointPositions endJP_;
     int pathResolution_;
     int measurementPerPoint_;
     int timeBetweenMeasurementsMS_;
     float forceThresholdN_;
     float torqueThresholdNm_;
     std::string logFile_;
};

}  // namespace ftsensorcalibrator
}  // namespace applications
}  // namespace crf
