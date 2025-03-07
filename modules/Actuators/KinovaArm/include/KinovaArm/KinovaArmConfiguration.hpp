#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>

#include "RobotArm/RobotArmConfiguration.hpp"

namespace crf::actuators::kinovaarm {

struct KinovaJacoNetworkConfiguration {
    std::string localAddressIP;
    std::string robotAddressIP;
    std::string subnetMask;
    unsigned int port;

    KinovaJacoNetworkConfiguration():
        localAddressIP(),
        robotAddressIP(),
        subnetMask(),
        port(0) {}
};

class KinovaArmConfiguration : public robotarm::RobotArmConfiguration {
 public:
    KinovaArmConfiguration();
    ~KinovaArmConfiguration() override = default;
    bool parse(const nlohmann::json& robotJSON) override;
    bool parse(const std::string&) = delete;

    /*
     * @brief
     * @return
     * @return
     */
    std::string getSerialNumber();
    /*
     * @brief
     * @return
     * @return
     */
    KinovaJacoNetworkConfiguration getNetworkConfiguration();

 protected:
    void cleanup() override;

 private:
    std::string serialNumber_;
    KinovaJacoNetworkConfiguration networkConfiguration_;
};

}  // namespace crf::actuators::kinovaarm
