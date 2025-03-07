/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include "RobotArm/RobotArmConfiguration.hpp"

namespace crf {
namespace robots {
namespace youbot {

class YoubotArmConfiguration : public robotarm::RobotArmConfiguration {
 public:
    KinovaArmConfiguration();
    ~KinovaArmConfiguration() override = default;
    bool parse(const nlohmann::json& robotJSON) override;
    bool parse(const std::string&) = delete;

    std::string getYoubotConfigurationFile();

 protected:
    void cleanup() override;

 private:
    std::string youbotConfigurationFile_;
};

}  // namespace youbot
}  // namespace robots
}  // namespace crf
