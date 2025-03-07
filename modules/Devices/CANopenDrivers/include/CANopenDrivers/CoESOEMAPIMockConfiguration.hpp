/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *         Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <bitset>
#include <condition_variable>
#include <cstring>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "SOEMAPI/SOEMAPIMockConfiguration.hpp"
#include "CANopenDrivers/CiA402/CiA402Definitions.hpp"
#include "CANopenDrivers/CiA402/CiA402Registers.hpp"
#include "CANopenDrivers/CiA301Registers.hpp"
#include "CANopenDrivers/RegistersSubIndexes.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

using crf::devices::canopendrivers::CiA402;
using crf::devices::canopendrivers::CiA301;
using crf::devices::canopendrivers::Subindex;
using crf::devices::canopendrivers::ControlWord;
using crf::devices::canopendrivers::StatusWord;
using crf::devices::canopendrivers::ModeOfOperation;
using crf::devices::canopendrivers::StatusWordMask;
using crf::devices::canopendrivers::ControlWordMask;

#pragma pack(push, 1)
struct TxPDO {
    uint16_t controlWord;
    int8_t modeOfOperation;
    uint8_t placeholder;
    int32_t targetPosition;
    int32_t targetVelocity;
    int16_t targetTorque;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RxPDO {
    uint16_t statusWord;
    int8_t modeOfOperationDisplay;
    uint8_t placeholder;
    int32_t positionActualValue;
    int32_t velocityActualValue;
    int16_t torqueActualValue;
    int16_t currentActualValue;
};
#pragma pack(pop)

namespace crf::devices::canopendrivers {

class CoESOEMAPIMockConfiguration : public crf::communication::soemapi::SOEMAPIMockConfiguration {
 public:
    CoESOEMAPIMockConfiguration() = delete;
    CoESOEMAPIMockConfiguration(const uint8_t& numberOfSlaves, const int& ioMapSize):
        SOEMAPIMockConfiguration(numberOfSlaves, ioMapSize),
        failToWriteSdo_(false),
        automaticTransitionFault_(false),
        shutdownFault_(false),
        switchOnFault_(false),
        disableVoltageFault_(false),
        quickStopFault_(false),
        disableOperationFault_(false),
        enableOperationFault_(false),
        logger_("CoESOEMAPIMockConfiguration") {
            coeInputs_.resize(numberOfSlaves_);
            coeOutputs_.resize(numberOfSlaves_);
            sdos_.resize(numberOfSlaves_);

            for (int i = 0; i < numberOfSlaves_; i++) {
                coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::NotReady);
            }
    }

    void configure() override {
        SOEMAPIMockConfiguration::configure();

        // RPDO
        createRegister(CiA301::RxPDOAssign, Subindex::SUB0);
        createRegister(CiA301::RxPDOAssign, Subindex::SUB1);
        createRegister(CiA301::RxPDOAssign, Subindex::SUB2);
        createRegister(CiA301::RxPDOAssign, Subindex::SUB3);
        createRegister(CiA301::RxPDOAssign, Subindex::SUB4);
        createRegister(CiA301::RxPDOAssign, Subindex::SUB5);
        createRegister(CiA301::RxPDOAssign, Subindex::SUB6);
        createRegister(CiA301::RxPDOAssign, Subindex::SUB7);
        createRegister(CiA301::RxPDOAssign, Subindex::SUB8);

        // TPDO
        createRegister(CiA301::TxPDOAssign, Subindex::SUB0);
        createRegister(CiA301::TxPDOAssign, Subindex::SUB1);
        createRegister(CiA301::TxPDOAssign, Subindex::SUB2);
        createRegister(CiA301::TxPDOAssign, Subindex::SUB3);
        createRegister(CiA301::TxPDOAssign, Subindex::SUB4);
        createRegister(CiA301::TxPDOAssign, Subindex::SUB5);
        createRegister(CiA301::TxPDOAssign, Subindex::SUB6);
        createRegister(CiA301::TxPDOAssign, Subindex::SUB7);
        createRegister(CiA301::TxPDOAssign, Subindex::SUB8);

        // CiA402
        createRegister(CiA402::ControlWord, Subindex::SUB0);
        createRegister(CiA402::StatusWord, Subindex::SUB0);
        createRegister(CiA402::ModesOfOperation, Subindex::SUB0);
        createRegister(CiA402::ModesOfOperationDisplay, Subindex::SUB0);
        createRegister(CiA402::SupportedDriveModes, Subindex::SUB0);
        createRegister(CiA402::MaxTorque, Subindex::SUB0);
        createRegister(CiA402::MaxCurrent, Subindex::SUB0);
        createRegister(CiA402::TargetTorque, Subindex::SUB0);
        createRegister(CiA402::TorqueOffset, Subindex::SUB0);
        createRegister(CiA402::PositionRangeLimit, Subindex::SUB1);
        createRegister(CiA402::PositionRangeLimit, Subindex::SUB2);
        createRegister(CiA402::SoftwarePositionLimit, Subindex::SUB1);
        createRegister(CiA402::SoftwarePositionLimit, Subindex::SUB2);
        createRegister(CiA402::Polarity, Subindex::SUB0);
        createRegister(CiA402::MaxProfileVelocity, Subindex::SUB0);
        createRegister(CiA402::MaxMotorSpeed, Subindex::SUB0);
        createRegister(CiA402::QuickStopDeceleration, Subindex::SUB0);
        createRegister(CiA402::MaxAcceleration, Subindex::SUB0);
        createRegister(CiA402::MaxDeceleration, Subindex::SUB0);
        createRegister(CiA402::MotionProfileType, Subindex::SUB0);
        createRegister(CiA402::EndVelocity, Subindex::SUB0);
        createRegister(CiA402::ProfileDeceleration, Subindex::SUB0);
        createRegister(CiA402::QuickStopDeceleration, Subindex::SUB0);
        createRegister(CiA402::MotorRatedTorque, Subindex::SUB0);
        createRegister(CiA402::InterpolationTimePeriod, Subindex::SUB1);
        createRegister(CiA402::InterpolationTimePeriod, Subindex::SUB2);
        createRegister(CiA402::VelocityOffset, Subindex::SUB0);

        ON_CALL(*this, config_overlap_map(_)).WillByDefault(Invoke(
            [this](void* pIOmap) {
                // Map slaves to their respective inputs and outputs
                for (int i = 0; i < numberOfSlaves_; i++) {
                    slaves_[i + 1].inputs = reinterpret_cast<uint8*>(&coeInputs_[i]);
                    slaves_[i + 1].outputs = reinterpret_cast<uint8*>(&coeOutputs_[i]);
                }

                if (stateForced_) return ioMapSize_;
                // Move slaves to SAFE-OPERATIONAL
                for (int i = 0; i < numberOfSlaves_ + 1; i++) {
                    slaves_[i].state = EC_STATE_SAFE_OP;
                }
                return ioMapSize_;
            }));

        ON_CALL(*this, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(
            Invoke([this](uint16_t slave, uint16_t Index, uint8_t SubIndex, bool CA, int psize, void *p, int Timeout) {  // NOLINT
                if (failToWriteSdo_) return 0;

                uint32_t idx = (((uint32_t)Index & 0x00FF) << 16) + ((uint32_t)SubIndex & 0x000F);
                if (sdos_[slave - 1].find(idx) == sdos_[slave - 1].end()) {
                    logger_->warn("Accessing not existing register");
                    return 0;
                }
                sdos_[slave - 1][idx] = std::string((char*)p, psize);  // NOLINT
                return 1;
        }));

        ON_CALL(*this, SDOread(_, _, _, _, _, _, _)).WillByDefault(
            Invoke([this](uint16_t slave, uint16_t index, uint8_t subindex, bool CA, int *psize, void *p, int timeout) {  // NOLINT
                auto sdoMap = sdos_[slave - 1];
                uint32_t idx = (((uint32_t)index & 0x00FF) << 16) + ((uint32_t)subindex & 0x000F);
                if (sdoMap.find(idx) == sdoMap.end()) {
                    logger_->warn("Accessing not existing register");
                    return 0;
                }
                auto value = sdoMap.at(idx);
                std::memcpy(p, value.c_str(), *psize);
                return 1;
        }));

        ON_CALL(*this, send_overlap_processdata()).WillByDefault(Invoke([this](){
            for (int i = 0; i < numberOfSlaves_; i++) {
                if (automaticTransitionFault_) {
                    goIntoFault(i);
                    return 0;
                }

                if (coeInputs_[i].statusWord == static_cast<uint16_t>(StatusWord::NotReady)) {
                    coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::SwitchONDisabled);
                }

                if ((coeOutputs_[i].controlWord & ControlWordMask::Shutdown) ==
                    ControlWord::Shutdown) {
                    if (shutdownFault_) {
                        goIntoFault(i);
                        return 0;
                    }
                    coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::Ready);
                }

                if (((coeOutputs_[i].controlWord & ControlWordMask::SwitchON) ==
                    ControlWord::SwitchON) &&
                    coeInputs_[i].statusWord == static_cast<uint16_t>(StatusWord::Ready)) {
                    if (switchOnFault_) {
                        goIntoFault(i);
                        return 0;
                    }
                    coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::SwitchedON);
                }

                if ((coeOutputs_[i].controlWord & ControlWordMask::DisableVoltage) ==
                    ControlWord::DisableVoltage) {
                    if (disableVoltageFault_) {
                        goIntoFault(i);
                        return 0;
                    }
                    coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::SwitchONDisabled);
                }

                if ((coeOutputs_[i].controlWord & ControlWordMask::QuickStop) ==
                    ControlWord::QuickStop) {
                    if (quickStopFault_) {
                        goIntoFault(i);
                        return 0;
                    }
                    coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::QuickStopActive);
                }

                if (((coeOutputs_[i].controlWord & ControlWordMask::DisableOperation) ==
                    ControlWord::DisableOperation) && coeInputs_[i].statusWord ==
                    static_cast<uint16_t>(StatusWord::OperationEnabled)) {
                    if (disableOperationFault_) {
                        goIntoFault(i);
                        return 0;
                    }
                    coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::SwitchedON);
                }

                if ((coeOutputs_[i].controlWord & ControlWordMask::EnableOperation) ==
                    ControlWord::EnableOperation) {
                    if (enableOperationFault_) {
                        goIntoFault(i);
                        return 0;
                    }
                    coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::OperationEnabled);
                }

                if ((coeOutputs_[i].controlWord & ControlWordMask::FaultResetActive) ==
                    ControlWord::FaultResetActive) {
                    coeInputs_[i].statusWord = static_cast<uint16_t>(StatusWord::SwitchONDisabled);
                }

                coeInputs_[i].modeOfOperationDisplay = coeOutputs_[i].modeOfOperation;
                coeInputs_[i].velocityActualValue = coeOutputs_[i].targetVelocity;
                coeInputs_[i].positionActualValue = coeOutputs_[i].targetPosition;
                coeInputs_[i].torqueActualValue = coeOutputs_[i].targetTorque;
            }

            return 0;
        }));
    }

    void goIntoFault(uint16_t slaveID) {
        coeInputs_[slaveID].statusWord = static_cast<uint16_t>(StatusWord::FaultReactionActive);
        coeInputs_[slaveID].statusWord = static_cast<uint16_t>(StatusWord::Fault);
    }

    void goIntoFaultShutdown() {
        logger_->warn("Shutdown fault");
        shutdownFault_ = true;
    }

    void goIntoFaultSwitchOn() {
        logger_->warn("Switch On fault");
        switchOnFault_ = true;
    }

    void goIntoFaultDisableVoltage() {
        logger_->warn("Disable voltage fault");
        disableVoltageFault_ = true;
    }

    void goIntoFaultQuickStop() {
        logger_->warn("Quick stop fault");
        quickStopFault_ = true;
    }

    void goIntoFaultDisableOperation() {
        logger_->warn("Disable operation fault");
        disableOperationFault_ = true;
    }

    void goIntoFaultEnableOperation() {
        logger_->warn("Enable operation fault");
        enableOperationFault_ = true;
    }

    void goIntoFaultAutomaticTransition() {
        logger_->warn("Automatic transition fault");
        automaticTransitionFault_ = true;
    }
    void resetValueTransitions() {
        logger_->warn("Reset values");
        coeOutputs_[0].controlWord = 0x0000;
        automaticTransitionFault_ = false;
        shutdownFault_ = false;
        switchOnFault_ = false;
        disableVoltageFault_ = false;
        quickStopFault_ = false;
        disableOperationFault_ = false;
        enableOperationFault_ = false;
    }

 protected:
    std::vector<RxPDO> coeInputs_;
    std::vector<TxPDO> coeOutputs_;

    std::vector<std::map<uint32_t, std::string>> sdos_;

    bool failToWriteSdo_;
    bool automaticTransitionFault_;
    bool shutdownFault_;
    bool switchOnFault_;
    bool disableVoltageFault_;
    bool quickStopFault_;
    bool disableOperationFault_;
    bool enableOperationFault_;

    crf::utility::logger::EventLogger logger_;

 private:
    void createRegister(uint16_t index, uint8_t subindex) {
        uint32_t idx =
            (((uint32_t)index & 0x00FF) << 16) + ((uint32_t)subindex & 0x000F);

        std::string tmp = "    ";
        for (int i=0; i < numberOfSlaves_; i++) {
            sdos_[i].insert({idx, tmp});
        }
    }
};

}  // namespace crf::devices::canopendrivers
