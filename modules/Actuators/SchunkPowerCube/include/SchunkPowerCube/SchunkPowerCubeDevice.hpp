#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>

#include "CANSocket/ICANSocket.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace robots {
namespace schunkpowercube {

class SchunkPowerCubeDevice : commoninterfaces::IInitializable {
 public:
    SchunkPowerCubeDevice() = delete;
    SchunkPowerCubeDevice(uint8_t canId,
        std::shared_ptr<communication::cansocket::ICANSocket> socket);
    SchunkPowerCubeDevice(const SchunkPowerCubeDevice&) = delete;
    SchunkPowerCubeDevice(SchunkPowerCubeDevice&&) = delete;
    ~SchunkPowerCubeDevice() override;

    bool initialize() override;
    bool deinitialize() override;

    bool setPosition(float angle);
    bool setVelocity(float velocity);

    bool setProfileVelocity(float velocity);
    bool setProfileAcceleration(float acceleration);

    bool getVelocity();
    bool getPosition();
    bool getCurrent();

    bool quitError();
    bool fastStop();
    bool getStatus();

    bool reset();

 private:
    utility::logger::EventLogger logger_;
    uint8_t canId_;
    std::shared_ptr<communication::cansocket::ICANSocket> socket_;

    bool initialized_;
};

}  // namespace schunkpowercube
}  // namespace robots
}  // namespace crf
