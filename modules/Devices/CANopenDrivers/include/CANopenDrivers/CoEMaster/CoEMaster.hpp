/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <string>
#include <optional>
#include <future>
#include <vector>
#include <sstream>

#include <nlohmann/json.hpp>

#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"
#include "CANopenDrivers/CiA301Registers.hpp"
#include "CANopenDrivers/SDOAbortCodeMessage.hpp"

#include "EventLogger/EventLogger.hpp"
#include "crf/expected.hpp"

using crf::communication::soemapi::ISOEMAPI;

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_coe_master
 * @brief The CANopen Over EtherCAT (CoE) master takes care of all the CANopen standard to
 * initialize the slaves in the network and allow for a seemless control.
 */
class CoEMaster : public ethercatdrivers::EtherCATMaster {
 public:
    CoEMaster(
        const std::string& ifname,
        const uint& numberOfSlaves,
        const std::chrono::microseconds& cycleTime,
        const nlohmann::json& slavesConfig,
        const int& sizeOfIOMap = 4096,
        std::shared_ptr<ISOEMAPI> soemApi = nullptr);
    ~CoEMaster() override;

    /**
     * @brief Sends a read SDO request. This function is used when an entity wants to send and SDO in
     *        order to read the Object Dictionary of a specific physical device.
     * @param SlaveID is the number of the slave in the EtherCAT Network.
     * @param Index is the one of the register of the Object Dictionary.
     * @param SubIndex is the one of the register of the Object Dictionary to read.
     * @param Size is the size (in byte) of the value to be read.
     * @param CA is used for the CompleteAccess reading. In this mode, all the subindex of the
     *        specified index are read.
     * @return The read value if the reading is successful (i.e. if the working counter returned by
     *         the function is equal to 1).
     * @return False otherwise.
     */
    template<typename T>
    crf::expected<T> readSDO(uint16_t slaveID, uint16_t index, uint8_t subIndex) const {
        int wkc = 0;
        T val;
        int size = sizeof(val);
        wkc = soemApi_->SDOread(slaveID, index, subIndex, false, &size, &val, EC_TIMEOUTRXM);
        if (wkc == 1) return val;

        ec_errort error;
        ec_poperror(&error);
        if (error.Etype != EC_ERR_TYPE_SDO_ERROR) {
            logger_->error("ReadSDO Error type: 0x{:04X}", error.Etype);
            logger_->error("ReadSDO Error code: 0x{:04X}", error.ErrorCode);
            logger_->error("ReadSDO Error register: 0x{:02X}", error.ErrorReg);
            return crf::ResponseCode(crf::Code::SDOReadAbort, error.AbortCode);
        }

        std::string description = "No Description";
        if (SDOAbortCodeMessage.find(error.AbortCode) != SDOAbortCodeMessage.end()) {
            description = SDOAbortCodeMessage.at(error.AbortCode);
        }
        if (description == "No Error") {
            logger_->warn("ReadSDO was succesfull but no further action was taken by the device");
            return val;
        }
        logger_->error("WKC should be 1 but it's: {}", wkc);
        logger_->error("ReadSDO failed on slave 0x{:04X}, index 0x{:04X}, subindex 0x{:02X}",
            slaveID, index, subIndex);
        logger_->error("ReadSDO abort type 0x{:04X}, abort code 0x{:04X}: \"{}\"",
            error.Etype , error.AbortCode, description);
        return crf::ResponseCode(crf::Code::SDOReadAbort, error.AbortCode);
    }

    /**
     * @brief Sends a Write SDO request. This function is used when an entity wants to send and SDO
     *        in order to write a value in the Object Dictionary of a specific physical device.
     * @param SlaveID is the number of the slave in the EtherCAT Network.
     * @param Index is the one of the register of the Object Dictionary.
     * @param SubIndex is the one of the register of the Object Dictionary to write.
     * @param BufferValue is the value that we want to write.
     * @param CA is used for the CompleteAccess writing. In this mode, all the subindex of the
     *        specified index are written.
     * @return True if the writing is successful (i.e. if the working counter returned by the
     *         function is equal to 1).
     * @return False otherwise.
     */
    template<typename T>
    crf::expected<bool> writeSDO(uint16_t slaveID, uint16_t index, uint8_t subIndex, T value) {
        int wkc = 0;
        int size = sizeof(value);
        wkc = soemApi_->SDOwrite(slaveID, index, subIndex, false, size, &value, EC_TIMEOUTRXM);
        if (wkc == 1) return true;

        ec_errort error;
        ec_poperror(&error);
        if (error.Etype != EC_ERR_TYPE_SDO_ERROR) {
            logger_->error("WriteSDO Error type: 0x{:04X}", error.Etype);
            logger_->error("WriteSDO Error code: 0x{:04X}", error.ErrorCode);
            logger_->error("WriteSDO Error register: 0x{:02X}", error.ErrorReg);
            return crf::ResponseCode(crf::Code::SDOWriteAbort, error.AbortCode);
        }

        std::string description = "No Description";

        if (SDOAbortCodeMessage.find(error.AbortCode) != SDOAbortCodeMessage.end()) {
            description = SDOAbortCodeMessage.at(error.AbortCode);
        }
        if (description == "No Error") {
            logger_->warn("WriteSDO was succesfull but no further action was taken by the device");
            return true;
        }
        logger_->error("WKC should be 1 but it's: {}", wkc);
        logger_->error("WriteSDO failed on slave 0x{:04X}, index 0x{:04X}, subindex 0x{:02X}, value 0x{:04x}",  // NOLINT
            slaveID, index, subIndex, value);
        logger_->error("WriteSDO abort type 0x{:04X}, abort code 0x{:04X}: \"{}\"",
            error.Etype , error.AbortCode, description);
        return crf::ResponseCode(crf::Code::SDOWriteAbort, error.AbortCode);
    }

 protected:
    bool onPreOperational() override;

 private:
    std::vector<nlohmann::json> slavesConfig_;
    utility::logger::EventLogger logger_;

    template<typename T>
    crf::expected<bool> writeSDOAndParse(
        uint16_t slaveID, uint16_t index, uint8_t subIndex, std::string valueString) {
        T value;
        std::stringstream ss;
        ss << std::hex << valueString;
        ss >> value;

        return writeSDO<T>(slaveID, index, subIndex, value);
    }
};

}  // namespace crf::devices::canopendrivers
