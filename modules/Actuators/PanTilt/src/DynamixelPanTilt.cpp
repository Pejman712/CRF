/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <utility>
#include <string>
#include <stdexcept>
#include <memory>

#include "PanTilt/DynamixelPanTilt.hpp"

namespace crf {
namespace devices {
namespace pantilt {

DynamixelPanTilt::DynamixelPanTilt(
    std::shared_ptr<crf::devices::dynamixelstepper::IDynamixel> dxl) :
    logger_("DynamixelPanTilt"), dxlChain_(dxl), initialized_(false) {logger_->debug("Ctor");}

bool DynamixelPanTilt::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Dynamixel set already initialized");
        return false;
    }
    if (!dxlChain_->initialize()) {
        logger_->error("Problem to initialize Dynamixel Control Software");
        return false;
    }
    if (dxlChain_->getConfiguration().getNumberOfMotors() != 2) {
        logger_->error("The number of motors must be exactly 2");
        return false;
    }
    initialized_ = true;
    auto zPos = dxlChain_->getZeroPosition();
    currentPosition_ = std::make_pair(zPos[0], zPos[1]);
    if (!setPosition(std::pair<float, float>(-1, -1))) {
        logger_->error("Problem to set the initial position");
        return false;
    }
    return true;
}

bool DynamixelPanTilt::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("The PanTilt was already deinitialized");
        return false;
    }
    if (!dxlChain_->deinitialize()) {
        logger_->error("Problem to deinitialize Dynamixel Control Software");
        return false;
    }
    initialized_ = false;
    return true;
}

std::pair<float, float> DynamixelPanTilt::getPosition() {
    logger_->debug("getPosition");
    if (!initialized_) {
        logger_->warn("The PanTilt has to be initialized");
        return {-1, -1};
    }
    auto degreeToRadian = [this](uint8_t i, uint16_t position) {
        auto motor = dxlChain_->getConfiguration().getMotorsChain()[i];
        return (motor.unitDegree * position * M_PI) / 180;
    };
    std::pair<float, float> currentPos(degreeToRadian(0, currentPosition_.first),
        degreeToRadian(1, currentPosition_.second));
    return currentPos;
}

void DynamixelPanTilt::computesTheMovement(uint8_t it, const int & coordinate,
    const crf::devices::dynamixelstepper::DXLParameter & dxl_param) {
    logger_->debug("computesTheMovement");
    if (coordinate < -1) {  // -1 means the begining, motors are send to zero position
        logger_->warn("Bad coordinate: {0}", coordinate);
        return;
    }
    switch (it) {
        case 0:
            if (coordinate < 0) currentPosition_.first = dxl_param.limits.DXLMinPosValue;
            else
                currentPosition_.first = (coordinate > dxl_param.limits.DXLMaxPosValue) ?
                    dxl_param.limits.DXLMaxPosValue : coordinate;
            break;
        case 1:
            if (coordinate < 0) currentPosition_.second = dxl_param.limits.DXLMinPosValue;
            else
                currentPosition_.second = (coordinate > dxl_param.limits.DXLMaxPosValue) ?
                    dxl_param.limits.DXLMaxPosValue : coordinate;
            break;
    }
}

bool DynamixelPanTilt::setPosition(const std::pair<float, float>& coordinates) {
    logger_->debug("setPosition");
    if (!initialized_) {
        logger_->warn("The PanTilt has to be initialized");
        return false;
    }
    auto configuration = dxlChain_->getConfiguration();
    auto radianToDegree = [&configuration](uint8_t i, float radians) {
        auto motor = configuration.getMotorsChain()[i];
        return (radians * 180) / (M_PI * motor.unitDegree);
    };
    for (int i = 0; i < dxlChain_->getConfiguration().getNumberOfMotors(); i++) {
        if (coordinates.first != -1 || coordinates.second != -1) {
            int degrees = (i == 0) ? radianToDegree(i, coordinates.first) :
                radianToDegree(i, coordinates.second);
            computesTheMovement(i, degrees, configuration.getMotorsChain()[i]);
        }
        uint16_t position = (i == 0) ? currentPosition_.first : currentPosition_.second;
        if (!dxlChain_->writeDynamixel(i, position)) {
            logger_->warn("Failed moving the motor");
            return false;
        }
    }
    return true;
}

}  // namespace pantilt
}  // namespace devices
}  // namespace crf
