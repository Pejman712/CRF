/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include "Robot/RobotConfiguration.hpp"

namespace crf::actuators::robot {

class Virtuose6DTAOConfiguration : public RobotConfiguration {
 public:
    explicit Virtuose6DTAOConfiguration(const nlohmann::json& robotJSON);
    explicit Virtuose6DTAOConfiguration(const std::string&) = delete;
    ~Virtuose6DTAOConfiguration() override = default;

    /**
     * @brief Get the IP address of the robot
     *
     * @return std::string
     */
    std::string getIPAddress();

 private:
    std::string ipAddress_;
    crf::utility::logger::EventLogger logger_;

    void parse(const nlohmann::json& robotJSON);
};

}  // namespace crf::actuators::robot
