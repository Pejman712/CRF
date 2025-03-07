#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace dynamixelstepper {

struct DXLLimits {
    uint16_t DXLMinPosValue;
    uint16_t DXLMaxPosValue;
    uint16_t DXLStartValue;
};

struct  AddrMx {
    uint16_t torqueEnable;
    uint16_t goalPosition;
    uint16_t presentPosition;
    uint16_t moving;
};

struct DXLParameter {
    uint8_t dxl_id;
    std::string mType;
    DXLLimits limits;
    uint16_t pulseMovement;
    double unitDegree;
    AddrMx addrMx;
    uint32_t baudRate;
    float protocolVersion;
};

class DynamixelConfiguration {
 public:
    DynamixelConfiguration();
    ~DynamixelConfiguration() = default;

    bool parse(const nlohmann::json &);

    inline std::vector<DXLParameter> getMotorsChain() const { return motorsChain_; }
    inline uint8_t getNumberOfMotors() const { return motorsChain_.size(); }

 private:
    utility::logger::EventLogger logger_;
    int amountOfMotors_;

    std::vector<DXLParameter> motorsChain_;
};

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
