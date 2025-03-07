/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Henry Paul Espinosa Peralta CERN BE/CEM/MRO 2023
 *         Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "IMU/Gable/SE1/GableSE1.hpp"

namespace crf::sensors::imu {

GableSE1::GableSE1(std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master,
    const uint16_t& id) :
    GableAbstract(master, id),
    rxpdo_(),
    txpdo_() {
}

IMUSignals GableSE1::getSignal() {
    logger_->debug("getSignals");
    if (!initialized_ || !isAlive()) {
        logger_->error("Device not initialized or device is not alive");
        IMUSignals output;
        output.position = crf::Code::NotInitialized;
        output.quaternion = crf::Code::NotInitialized;
        output.eulerZYX = crf::Code::NotInitialized;
        output.linearVelocity = crf::Code::NotInitialized;
        output.angularVelocity = crf::Code::NotInitialized;
        output.linearAcceleration = crf::Code::NotInitialized;
        output.angularAcceleration = crf::Code::NotInitialized;
        output.magneticField = crf::Code::NotInitialized;
        return output;
    }
    IMUSignals output;
    output.angularVelocity = std::array<double, 3>({rxpdo_->gx, rxpdo_->gy, rxpdo_->gz});
    output.linearAcceleration = std::array<double, 3>({rxpdo_->ax, rxpdo_->ay, rxpdo_->az});
    output.magneticField = std::array<double, 3>({rxpdo_->magX, rxpdo_->magY, rxpdo_->magZ});
    return output;
}

crf::expected<GableInfo> GableSE1::getInfo() {
    logger_->debug("getInfo");
    if (!initialized_ || !isAlive()) {
        logger_->error("Device not initialized or device is not alive");
        return crf::Code::NotInitialized;
    }
    GableInfo info;
    info.Baudrate = static_cast<unsigned int>(rxpdo_->Baudrate);
    info.Firmware = static_cast<int>(rxpdo_->Firmware);
    info.Hardware = static_cast<int>(rxpdo_->Hardware);
    info.FilterProfile = crf::Code::NotImplemented;
    info.DeviceID = static_cast<int>(rxpdo_->DeviceID);
    return info;
}

bool GableSE1::bindIOMap() {
    logger_->debug("bindIOMap");
    std::optional<uint8_t*> in = master_->retrieveInputs(getID());
    std::optional<uint8_t*> out = master_->retrieveOutputs(getID());
    if (!in || !out) {
        logger_->error("Can't retrieve Inputs or Outputs pointers from the IOMap");
        return false;
    }
    rxpdo_ = reinterpret_cast<GableRxPDO_SE1*>(in.value());
    txpdo_ = reinterpret_cast<GableTxPDO*>(out.value());
    logger_->info("IO Map Binded");
    return true;
}

}  // namespace crf::sensors::imu
