/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <exception>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "Dynamixel/DynamixelConfiguration.hpp"

namespace crf {
namespace devices {
namespace dynamixelstepper {

DynamixelConfiguration::DynamixelConfiguration() : logger_("DXL_config"), amountOfMotors_(0) {}

bool DynamixelConfiguration::parse(const nlohmann::json & dxl_JSON) {
    logger_->debug("parse");
    try {
        amountOfMotors_ = dxl_JSON["NumberOfMotors"].get<uint8_t>();

        for (int i = 0; i < amountOfMotors_; i++) {
            DXLParameter dxl_param;

            dxl_param.dxl_id = dxl_JSON.at("dxl_id")[i];
            dxl_param.mType = dxl_JSON.at("MotorType");
            dxl_param.pulseMovement = dxl_JSON.at("PulseMovement").get<uint16_t>();
            dxl_param.unitDegree = dxl_JSON.at("Unit_degree").get<double>();

            auto dxl_limits = dxl_JSON.at("Limits")[i];
            dxl_param.limits.DXLMinPosValue = dxl_limits.at("MinPosValue").get<uint16_t>();
            dxl_param.limits.DXLMaxPosValue = dxl_limits.at("MaxPosValue").get<uint16_t>();
            dxl_param.limits.DXLStartValue  = dxl_limits.at("StartValue").get<uint16_t>();

            auto dx_addr = dxl_JSON.at("AddrMX");
            dxl_param.addrMx.torqueEnable = dx_addr.at("TorqueEnable").get<uint16_t>();
            dxl_param.addrMx.goalPosition = dx_addr.at("GoalPosition").get<uint16_t>();
            dxl_param.addrMx.presentPosition = dx_addr.at("PresentPosition").get<uint16_t>();
            dxl_param.addrMx.moving = dx_addr.at("Moving").get<uint16_t>();

            dxl_param.baudRate = dxl_JSON.at("Baud_Rate").get<uint32_t>();
            dxl_param.protocolVersion = dxl_JSON.at("ProtocolVersion").get<float>();

            motorsChain_.push_back(dxl_param);
        }
    } catch (const std::exception& e) {
        logger_->error("Failed to parse because: {}", e.what());
        return false;
    }

    return true;
}

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
