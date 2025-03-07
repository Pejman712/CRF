/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <vector>

#include "RobotArm/RobotArmConfiguration.hpp"

namespace crf::actuators::universalrobot {

class UniversalRobotConfiguration : public robotarm::RobotArmConfiguration {
 public:
    UniversalRobotConfiguration();
    ~UniversalRobotConfiguration() override = default;
    bool parse(const nlohmann::json& robotJSON) override;
    bool parse(const std::string&) = delete;
    std::string getIPAddress();
    float getLookAheadTime();
    float getGain();

 protected:
    void cleanup() override;

 private:
    std::string ipAddress_;
    float lookAheadTime_;
    float gain_;
};

}  // namespace crf::actuators::universalrobot
