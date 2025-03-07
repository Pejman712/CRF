/*
 * © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Henry Paul Espinosa Peralta CERN BE/CEM/MRO 2023
 *         Giancarlo D'Ago CERN BE/CEM/MRO 2023
 * 
 *  ==================================================================================================
 */

#include "IMU/Gable/SE2SE3/GableSE2SE3.hpp"

namespace crf::sensors::imu {

GableSE2SE3::GableSE2SE3(std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master,
    const uint16_t& id) :
    GableAbstract(master, id),
    rxpdo_(),
    txpdo_() {
}

IMUSignals GableSE2SE3::getSignal() {
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
    output.quaternion = std::array<double, 4>({rxpdo_->qw, rxpdo_->qx, rxpdo_->qy, rxpdo_->qz});
    output.eulerZYX = std::array<double, 3>({rxpdo_->ex, rxpdo_->ey, rxpdo_->ez});
    output.angularVelocity = std::array<double, 3>({rxpdo_->gx, rxpdo_->gy, rxpdo_->gz});
    output.linearAcceleration = std::array<double, 3>({rxpdo_->ax, rxpdo_->ay, rxpdo_->az});
    output.magneticField = std::array<double, 3>({rxpdo_->magX, rxpdo_->magY, rxpdo_->magZ});
    return output;
}

crf::expected<GableInfo> GableSE2SE3::getInfo() {
    logger_->debug("getInfo");
    if (!initialized_ || !isAlive()) {
        logger_->error("Device not initialized or device is not alive");
        return crf::Code::NotInitialized;
    }
    // Request state transition to config mode, then send a "ReqFilterProfile" message (ID 100
    // + empty DATA), and finally request to transition back to measurement mode.
    if (!goToConfigMode() || !sendCustomMessage(100, {}) || !goToMeasurementMode()) {
        return crf::Code::BadRequest;
    }
    GableInfo info;
    info.Baudrate = static_cast<unsigned int>(rxpdo_->Baudrate);
    info.Firmware = static_cast<int>(rxpdo_->Firmware);
    info.Hardware = static_cast<int>(rxpdo_->Hardware);
    info.FilterProfile = static_cast<unsigned int>(rxpdo_->FilterProfile);
    info.DeviceID = static_cast<int>(rxpdo_->DeviceID);
    return info;
}

bool GableSE2SE3::enableAHS() {
    logger_->debug("enableAHS");
    // Request state transition to config mode. Then send a "SetOptionFlags" message
    // (ID 72 + 8 byte DATA) with optionMessage "EnableAhs" (0x00000010 = 0 0 0 16), to be
    // inserted in the first 4 bytes, i.e "SetFlags" part. The second 4 bytes, i.e. the
    // "clearFlags" part, of the DATA message is zero. Finally request to transition back
    // to measurement mode.
    if (!goToConfigMode() || !sendCustomMessage(72, {0, 0, 0, 16, 0, 0, 0, 0}) ||
        !goToMeasurementMode()) {
        return false;
    }
    return true;
}

bool GableSE2SE3::disableAHS() {
    logger_->debug("disableAHS");
    // Request state transition to config mode. Then send a "SetOptionFlags" message
    // (ID 72 + 8 byte DATA) with optionMessage "EnableAhs" (0x00000010 = 0 0 0 16), to be
    // inserted in the second 4 bytes, i.e "clearFlags" part. The first 4 bytes, i.e. the
    // "setFlags" part, of the DATA message is zero. Finally request to transition back
    // to measurement mode.
    if (!goToConfigMode() || !sendCustomMessage(72, {0, 0, 0, 0, 0, 0, 0, 16}) ||
        !goToMeasurementMode()) {
        return false;
    }
    return true;
}

bool GableSE2SE3::setFilter(FilterProfile profile) {
    logger_->debug("setFilter");
    // Request state transition to config mode. Then send a "SetFilterProfile" message
    // (ID 100 + 2 byte DATA). DATA message has to contain the desired filter in decimal.
    // Finally request to transition back to measurement mode.
    if (!goToConfigMode() || !sendCustomMessage(100, {0, static_cast<uint8_t>(profile)})
        || !goToMeasurementMode()) {
        return false;
    }
    return true;
}

bool GableSE2SE3::sendCustomMessage(const unsigned int mid, std::vector<unsigned int> dataVector) {
    logger_->debug("sendCustomMessage");
    unsigned int dataLength = dataVector.size();
    GableTxPDO dataPacket;
    dataPacket.Command = static_cast<uint8_t>(1);  // Command 1 corresponds to "CustomMessage mode"
    dataPacket.MID = static_cast<uint8_t>(mid);
    dataPacket.DataLength = static_cast<uint8_t>(dataLength);
    dataPacket.Data0 = static_cast<uint8_t>((dataLength > 0) ? dataVector[0] : 0);
    dataPacket.Data1 = static_cast<uint8_t>((dataLength > 1) ? dataVector[1] : 0);
    dataPacket.Data2 = static_cast<uint8_t>((dataLength > 2) ? dataVector[2] : 0);
    dataPacket.Data3 = static_cast<uint8_t>((dataLength > 3) ? dataVector[3] : 0);
    dataPacket.Data4 = static_cast<uint8_t>((dataLength > 4) ? dataVector[4] : 0);
    dataPacket.Data5 = static_cast<uint8_t>((dataLength > 5) ? dataVector[5] : 0);
    dataPacket.Data6 = static_cast<uint8_t>((dataLength > 6) ? dataVector[6] : 0);
    dataPacket.Data7 = static_cast<uint8_t>((dataLength > 7) ? dataVector[7] : 0);
    dataPacket.Data8 = static_cast<uint8_t>((dataLength > 8) ? dataVector[8] : 0);
    dataPacket.Data9 = static_cast<uint8_t>((dataLength > 9) ? dataVector[9] : 0);
    dataPacket.Data10 = static_cast<uint8_t>((dataLength > 10) ? dataVector[10] : 0);
    dataPacket.Data11 = static_cast<uint8_t>((dataLength > 11) ? dataVector[11] : 0);
    dataPacket.Data12 = static_cast<uint8_t>((dataLength > 12) ? dataVector[12] : 0);
    dataPacket.Data13 = static_cast<uint8_t>((dataLength > 13) ? dataVector[13] : 0);
    dataPacket.Data14 = static_cast<uint8_t>((dataLength > 14) ? dataVector[14] : 0);
    dataPacket.Data15 = static_cast<uint8_t>((dataLength > 15) ? dataVector[15] : 0);
    dataPacket.Data16 = static_cast<uint8_t>((dataLength > 16) ? dataVector[16] : 0);
    dataPacket.Data17 = static_cast<uint8_t>((dataLength > 17) ? dataVector[17] : 0);
    dataPacket.Data18 = static_cast<uint8_t>((dataLength > 18) ? dataVector[18] : 0);
    dataPacket.Data19 = static_cast<uint8_t>((dataLength > 19) ? dataVector[19] : 0);
    dataPacket.Data20 = static_cast<uint8_t>((dataLength > 20) ? dataVector[20] : 0);
    dataPacket.Data21 = static_cast<uint8_t>((dataLength > 21) ? dataVector[21] : 0);
    dataPacket.Data22 = static_cast<uint8_t>((dataLength > 22) ? dataVector[22] : 0);
    dataPacket.Data23 = static_cast<uint8_t>((dataLength > 23) ? dataVector[23] : 0);
    dataPacket.Data24 = static_cast<uint8_t>((dataLength > 24) ? dataVector[24] : 0);
    dataPacket.Data25 = static_cast<uint8_t>((dataLength > 25) ? dataVector[25] : 0);
    dataPacket.Data26 = static_cast<uint8_t>((dataLength > 26) ? dataVector[26] : 0);
    dataPacket.Data27 = static_cast<uint8_t>((dataLength > 27) ? dataVector[27] : 0);
    memcpy(txpdo_, &dataPacket, sizeof(GableTxPDO));
    // Waiting for acknowledgment of the custom message sent. The ack
    // corresponds to a message with the ID equal to the the ID of the message
    // sent, increased by 1
    auto start = std::chrono::high_resolution_clock::now();
    while (static_cast<unsigned int>(rxpdo_->CommandResponse) != mid+1) {
        // Check if an "Error" Message (ID 66) is received
        if (static_cast<unsigned int>(rxpdo_->CommandResponse) == 66) {
            logger_->error("Invalid command");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed > timeoutMs_) {
            logger_->error("Transition not successful, timeout");
            return false;
        }
    }
    return true;
}

bool GableSE2SE3::goToConfigMode() {
    logger_->debug("goToConfigMode");
    txpdo_->Command = static_cast<uint8_t>(2);  // Command 2 corresponds to "Config Mode"
    // Waiting for "GoToConfigAck" Message (ID 49)
    auto start = std::chrono::high_resolution_clock::now();
    while (static_cast<unsigned int>(rxpdo_->CommandResponse) != 49) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed > timeoutMs_) {
            logger_->error("Transition not successful, timeout");
            return false;
        }
    }
    return true;
}

bool GableSE2SE3::goToMeasurementMode() {
    logger_->debug("goToMeasurementMode");
    txpdo_->Command = static_cast<uint8_t>(3);  // Command 3 corresponds to "Measurement Mode"
    // Here we skip the receiving of the "GoToMeasurementAck" message (ID 17), but
    // directly wait for the first message "MTData2" to arrive (ID 54).
    auto start = std::chrono::high_resolution_clock::now();
    while (static_cast<unsigned int>(rxpdo_->CommandResponse) != 54) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (elapsed > timeoutMs_) {
            logger_->error("Transition not successful, timeout");
            return false;
        }
    }
    return true;
}

bool GableSE2SE3::bindIOMap() {
    logger_->debug("bindIOMap");
    std::optional<uint8_t*> in = master_->retrieveInputs(getID());
    std::optional<uint8_t*> out = master_->retrieveOutputs(getID());
    if (!in || !out) {
        logger_->error("Can't retrieve Inputs or Outputs pointers from the IOMap");
        return false;
    }
    rxpdo_ = reinterpret_cast<GableRxPDO_SE2SE3*>(in.value());
    txpdo_ = reinterpret_cast<GableTxPDO*>(out.value());
    logger_->info("IO Map Binded");
    return true;
}

}  // namespace crf::sensors::imu
