/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: David Forkel CERN BE/CEM/MRO
 *
 *  ==================================================================================================
 */


#pragma once

#include <string>
#include <vector>

#include "Robot/RobotConfiguration.hpp"

namespace crf::actuators::robot {

class EtherCATRobotConfiguration : public RobotConfiguration {
 public:
    explicit EtherCATRobotConfiguration(const nlohmann::json& robotConfig);
    explicit EtherCATRobotConfiguration(const std::string&) = delete;
    ~EtherCATRobotConfiguration() override = default;

    uint32_t getGearBoxReduction() const;
    uint32_t getMaxCurrent() const;
    uint32_t getMaxTorque() const;
    double getRadToCountRatio() const;

 private:
    void parse(const nlohmann::json& robotJSON);
    void parse(const std::string&) = delete;

    uint32_t gearBoxRatio_;
    uint32_t maxCurrent_;
    uint32_t maxTorque_;
    double radToCountRatio_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::robot
