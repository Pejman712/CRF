#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <nlohmann/json.hpp>

#include "Dynamixel/DynamixelConfiguration.hpp"
#include "Dynamixel/DynamixelSDK.hpp"
#include "Dynamixel/IDynamixel.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace dynamixelstepper {

class Dynamixel : public IDynamixel {
 public:
    Dynamixel(const std::string &, const nlohmann::json &,
      std::shared_ptr<IDynamixelSDK> = nullptr);
    Dynamixel(const Dynamixel& other) = delete;
    Dynamixel(Dynamixel&& other) = delete;
    Dynamixel() = delete;
    ~Dynamixel() override;

    bool initialize() override;
    bool deinitialize() override;

    bool writeDynamixel(uint8_t, uint16_t) override;
    int16_t readCurrentPosition(uint8_t) override;

    inline std::vector<uint16_t> getZeroPosition() override { return zero_position_; }
    inline DynamixelConfiguration getConfiguration() override { return configuration_; }

 private:
    utility::logger::EventLogger logger_;
    std::string port_;
    nlohmann::json dxl_ConfigFile_;
    int dxl_comm_result_;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    bool initialized_;
    std::shared_ptr<IDynamixelSDK> dynamixelSdk_;

    std::vector<uint16_t> zero_position_;
    DynamixelConfiguration configuration_;

    bool createHandlers();
    bool connectToDynamixel();
    bool closeConnection();
};

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
