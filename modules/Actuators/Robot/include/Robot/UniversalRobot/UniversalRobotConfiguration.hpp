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

#include "Robot/RobotConfiguration.hpp"

namespace crf::actuators::robot {

class UniversalRobotConfiguration : public RobotConfiguration {
 public:
    explicit UniversalRobotConfiguration(const nlohmann::json& robotConfig);
    explicit UniversalRobotConfiguration(const std::string&) = delete;
    ~UniversalRobotConfiguration() override = default;

    /**
     * @brief Get the IP address of the robot
     *
     * @return std::string
     */
    std::string getIPAddress() const;

    /**
     * @brief Get the Look Ahead Time of the robot
     *
     * @return double
     */
    double getLookAheadTime() const;

    /**
     * @brief Get the Gain of the robot
     *
     * @return double
     */
    double getGain() const;

 private:
    void parse(const nlohmann::json& robotJSON);
    void parse(const std::string&) = delete;

    std::string ipAddress_;
    double lookAheadTime_;
    double gain_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::robot
