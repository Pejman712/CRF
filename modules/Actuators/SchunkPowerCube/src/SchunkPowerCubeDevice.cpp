/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <linux/can.h>
#include <memory>

#include "SchunkPowerCube/SchunkPowerCubeDevice.hpp"

namespace crf {
namespace robots {
namespace schunkpowercube {

SchunkPowerCubeDevice::SchunkPowerCubeDevice(uint8_t canId,
    std::shared_ptr<communication::cansocket::ICANSocket> socket) :
    logger_("SchunkPowerCubeDevice-" + std::to_string(canId)),
    canId_(canId),
    socket_(socket),
    initialized_(false) {
    logger_->debug("CTor");
}

SchunkPowerCubeDevice::~SchunkPowerCubeDevice() {
    logger_->debug("CTor");
    if (initialized_)
        deinitialize();
}

bool SchunkPowerCubeDevice::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    initialized_ = true;
    return true;
}

bool SchunkPowerCubeDevice::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    initialized_ = false;
    return true;
}

bool SchunkPowerCubeDevice::setPosition(float angle) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    auto mangleChar = reinterpret_cast<unsigned char const*>(&angle);
    can_frame frame;
    frame.can_id = 0xE0 + canId_;
    frame.can_dlc = 0x06;
    frame.data[0] = 0x0B;
    frame.data[1] = 0x04;
    frame.data[2] = mangleChar[0];
    frame.data[3] = mangleChar[1];
    frame.data[4] = mangleChar[2];
    frame.data[5] = mangleChar[3];
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::setVelocity(float velocity) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    auto mvelocityChar = reinterpret_cast<unsigned char const*>(&velocity);
    can_frame frame;
    frame.can_id = 0xE0 + canId_;
    frame.can_dlc = 0x06;
    frame.data[0] = 0x0B;
    frame.data[1] = 0x07;
    frame.data[2] = mvelocityChar[0];
    frame.data[3] = mvelocityChar[1];
    frame.data[4] = mvelocityChar[2];
    frame.data[5] = mvelocityChar[3];
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::setProfileVelocity(float velocity) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    auto mvelocityChar = reinterpret_cast<unsigned char const*>(&velocity);
    can_frame frame;
    frame.can_id = 0xE0 + canId_;
    frame.can_dlc = 0x06;
    frame.data[0] = 0x08;
    frame.data[1] = 0x4F;
    frame.data[2] = mvelocityChar[0];
    frame.data[3] = mvelocityChar[1];
    frame.data[4] = mvelocityChar[2];
    frame.data[5] = mvelocityChar[3];
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::setProfileAcceleration(float acceleration) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    auto maccelerationChar = reinterpret_cast<unsigned char const*>(&acceleration);
    can_frame frame;
    frame.can_id = 0xE0 + canId_;
    frame.can_dlc = 0x06;
    frame.data[0] = 0x08;
    frame.data[1] = 0x50;
    frame.data[2] = maccelerationChar[0];
    frame.data[3] = maccelerationChar[1];
    frame.data[4] = maccelerationChar[2];
    frame.data[5] = maccelerationChar[3];
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::getVelocity() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    can_frame frame = {};
    frame.can_id = 0xC0+canId_;
    frame.can_dlc = 0x02;
    frame.data[0] = 0x0A;
    frame.data[1] = 0x41;
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::getPosition() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    can_frame frame = {};
    frame.can_id = 0xC0+canId_;
    frame.can_dlc = 0x02;
    frame.data[0] = 0x0A;
    frame.data[1] = 0x3C;
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::getCurrent() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    can_frame frame = {};
    frame.can_id = 0xC0+canId_;
    frame.can_dlc = 0x02;
    frame.data[0] = 0x0A;
    frame.data[1] = 0x4D;
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::getStatus() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    can_frame frame = {};
    frame.can_id = 0xC0+canId_;
    frame.can_dlc = 0x02;
    frame.data[0] = 0x0A;
    frame.data[1] = 0x27;
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::quitError() {
    logger_->debug("quitError");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    can_frame frame = {};
    frame.can_id = 0xE0+canId_;
    frame.can_dlc = 0x01;
    frame.data[0] = 0x11;
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::fastStop() {
    logger_->debug("fastStop");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    can_frame frame = {};
    frame.can_id = 0xE0+canId_;
    frame.can_dlc = 0x01;
    frame.data[0] = 0x11;
    return socket_->write(&frame) == 16;
}

bool SchunkPowerCubeDevice::reset()  {
    logger_->debug("reset");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    can_frame frame = {};
    frame.can_id = 0xE0+canId_;
    frame.can_dlc = 0x01;
    frame.data[0] = 0x00;
    return socket_->write(&frame) == 16;
}

}  // namespace schunkpowercube
}  // namespace robots
}  // namespace crf
