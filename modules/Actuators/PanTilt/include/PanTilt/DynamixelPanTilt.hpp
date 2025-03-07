#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <utility>
#include <memory>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "PanTilt/IPanTilt.hpp"
#include "Dynamixel/Dynamixel.hpp"

namespace crf {
namespace devices {
namespace pantilt {

class DynamixelPanTilt : public IPanTilt {
 public:
    explicit DynamixelPanTilt(std::shared_ptr<crf::devices::dynamixelstepper::IDynamixel>);
    DynamixelPanTilt(const DynamixelPanTilt& other) = delete;
    DynamixelPanTilt(DynamixelPanTilt&& other) = delete;
    ~DynamixelPanTilt() = default;

    bool initialize() override;
    bool deinitialize() override;
    std::pair<float, float> getPosition() override;
    bool setPosition(const std::pair<float, float> &) override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<crf::devices::dynamixelstepper::IDynamixel> dxlChain_;
    bool initialized_;
    std::pair<uint16_t, uint16_t> currentPosition_;

    /**
    * [DynamixelPanTilt::computesTheMovement description]
    * @param it         iterator for motor
    * @param coordinate goal position
    * @param dxl_param  motor parameters
    */
    void computesTheMovement(uint8_t,
        const int &,
        const crf::devices::dynamixelstepper::DXLParameter &);
};

}  // namespace pantilt
}  // namespace devices
}  // namespace crf
