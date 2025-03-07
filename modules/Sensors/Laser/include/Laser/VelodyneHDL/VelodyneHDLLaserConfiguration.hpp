/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero & Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "Laser/LaserConfiguration.hpp"

namespace crf {
namespace sensors {
namespace laser {

struct VelodyneHDLLaserNetworkConfiguration {
    std::string ipAddress;
    uint16_t udpPort;

    VelodyneHDLLaserNetworkConfiguration():
        ipAddress(),
        udpPort(0) {}
};

class VelodyneHDLLaserConfiguration : public laser::LaserConfiguration  {
 public:
    VelodyneHDLLaserConfiguration();
    ~VelodyneHDLLaserConfiguration() override = default;
    bool parse(const nlohmann::json& laserJSON)  override;
    bool parse(const std::string&) = delete;

    VelodyneHDLLaserNetworkConfiguration getNetworkConfiguration();
    std::string getCorrectionParametersFilePath();

 protected:
    void cleanUp() override;

 private:
    VelodyneHDLLaserNetworkConfiguration networkConfiguration_;
    std::string correctionParametersFilePath_;
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
