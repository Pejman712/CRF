#pragma once
/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 * Contributor: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <map>

#include "RobotArm/RobotArmConfiguration.hpp"
#include "KinovaArm/KinovaArmConfiguration.hpp"

namespace crf::actuators::timarm {

class TIMArmConfiguration : public robotarm::RobotArmConfiguration {
 public:
    TIMArmConfiguration();
    ~TIMArmConfiguration() override = default;
    bool parse(const nlohmann::json& robotJSON) override;
    bool parse(const std::string&) = delete;

    std::string getKinovaSerialNumber();
    unsigned int getKinovaNumberOfJoints();
    unsigned int getHarmonicNumberOfJoints();
    crf::actuators::kinovaarm::KinovaJacoNetworkConfiguration getKinovaNetworkConfiguration();

    nlohmann::json getKinovaArmJSON();
    nlohmann::json getHarmonicArmJSON();

 protected:
    void cleanup() override;

 private:
    /*
     * Creates a JSON object that can be use to create a KinovaJaco object.
     */
    void generateKinovaArmJSON();
    /*
     * Creates a JSON object that can be use to create a HarmonicArm object.
     */
    void generateHarmonicArmJSON();

    std::string kinovaSerialNumber_;
    unsigned int kinovaJointsNumber_;
    unsigned int etherCATJointsNumber_;
    crf::actuators::kinovaarm::KinovaJacoNetworkConfiguration kinovaNetworkConfiguration_;
    nlohmann::json kinovaArmJSON_;
    nlohmann::json etherCATArmJSON_;
    std::map<crf::actuators::robotarm::DHParameter::JointType, std::string> mapJointType_;
};

}  // namespace crf::actuators::timarm
