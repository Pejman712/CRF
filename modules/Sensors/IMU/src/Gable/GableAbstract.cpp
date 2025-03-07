/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Henry Paul Espinosa Peralta CERN BE/CEM/MRO 2023
 *         Giancarlo D'Ago CERN BE/CEM/MRO 2023
 * 
 *  ==================================================================================================
 */

#include "IMU/Gable/GableAbstract.hpp"

namespace crf::sensors::imu {

GableAbstract::GableAbstract(std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master,
    const uint16_t& id) :
    BasicEtherCATDriver(master, id),
    master_(master),
    logger_("Gable"),
    imuData_{} {
    logger_->debug("CTor");
}

GableAbstract::~GableAbstract() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool GableAbstract::initialize() {
    logger_->debug("initialize");
    return BasicEtherCATDriver::initialize();
}

bool GableAbstract::deinitialize() {
    logger_->debug("Deinitialize");
    return BasicEtherCATDriver::deinitialize();
}

crf::expected<bool> GableAbstract::calibrate() {
    logger_->debug("calibrate");
    if (!initialized_ || !isAlive()) {
        logger_->error("Device not initialized or device is not alive");
        return crf::Code::NotInitialized;
    }
    return crf::Code::MethodNotAllowed;
}

}  // namespace crf::sensors::imu
