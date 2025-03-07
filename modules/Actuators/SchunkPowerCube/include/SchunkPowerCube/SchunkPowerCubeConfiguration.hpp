#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>

#include "RobotArm/RobotArmConfiguration.hpp"

namespace crf {
namespace robots {
namespace schunkpowercube {

class SchunkPowerCubeConfiguration : public robotarm::RobotArmConfiguration {
 public:
    SchunkPowerCubeConfiguration();
    ~SchunkPowerCubeConfiguration() override = default;

    bool parse(const nlohmann::json& robotJSON);
    bool parse(const std::string&) = delete;
    std::vector<int> getJointsCanID();

 protected:
    void cleanup() override;

 private:
    std::vector<int> jointsCanID_;
};

}  // namespace schunkpowercube
}  // namespace robots
}  // namespace crf
