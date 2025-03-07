/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace laser {

struct FrameLimits {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
};

struct LaserParameters {
    float minAngle;
    float maxAngle;
    float minRange;
    float maxRange;
    unsigned int scanSize;
    unsigned int frequency;
    unsigned int LidarVendorID;
    unsigned int LidarProductID;
};

class LaserConfiguration {
 public:
    LaserConfiguration();
    virtual ~LaserConfiguration() = default;

    virtual bool parse(const nlohmann::json& laserJSON);
    bool parse(const std::string&) = delete;

    bool setLaserParameters(const LaserParameters& laserParameters);
    LaserParameters getLaserParameters();

    FrameLimits getFrameLimits(); 
    
    bool setScanIntervalUS(const std::chrono::microseconds& interval);
    std::chrono::microseconds getScanIntervalUS();
    
    
    
    float getAngularResolution();

 protected:
    utility::logger::EventLogger logger_;
    LaserParameters laserParameters_;
    FrameLimits framelimits_;
    float angularResolution_;
    std::chrono::microseconds scanInterval_;
    virtual void cleanUp();
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
