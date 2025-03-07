/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#pragma once

#include <string>
#include <vector>

#include "Robot/RobotConfiguration.hpp"

namespace crf::actuators::robot {

/**
 * @brief Struct to group the paraemters that describe the network
 * configuration of a Kinova Jaco 2 robot.
 *
 */
struct KinovaJaco2NetworkConfiguration {
    std::string localAddressIP;
    std::string robotAddressIP;
    std::string subnetMask;
    uint64_t port;
};

class KinovaJaco2Configuration : public RobotConfiguration {
 public:
    explicit KinovaJaco2Configuration(const nlohmann::json& robotConfig);
    explicit KinovaJaco2Configuration(const std::string&) = delete;
    ~KinovaJaco2Configuration() override = default;

    /**
     * @brief Method to retrieve the serial number present in the configuration file
     *
     * @return std::string
     */
    std::string getSerialNumber() const;

    /**
     * @brief Method to retrieve the network configuration of the kinova arm
     *
     * @return KinovaJaco2NetworkConfiguration
     */
    KinovaJaco2NetworkConfiguration getNetworkConfiguration() const;

 private:
    void parse(const nlohmann::json& robotJSON);
    void parse(const std::string&) = delete;

    std::string serialNumber_;
    KinovaJaco2NetworkConfiguration networkConfiguration_;

    crf::utility::logger::EventLogger logger_;
};

}  // namespace crf::actuators::robot
