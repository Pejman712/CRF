/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
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

#include "EtherCATDevices/SoemApiMock.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

struct EC_output {
    int32_t targetPosition;
    int32_t targetVelocity;
    int16_t targetTorque;
    uint16_t maxTorque;
    uint16_t controlWord;
    int8_t modeOfOperation;
    uint32_t digitalOutputs;
};

struct EC_input {
    int32_t positionActualValue;
    uint32_t digitalInputs;
    int32_t velocityActualValue;
    uint16_t statusWord;
    int8_t modeOfOperationDisplay;
    int16_t torqueActualValue;
    int16_t analogInput;
    int16_t currentActualValue;
};

using crf::devices::ethercatdevices::SoemApiMock;

class SoemSimulator {
 public:
    SoemSimulator() = delete;
    SoemSimulator(std::shared_ptr<SoemApiMock> soemMock, uint8_t numberOfSlaves) :
        soemMock_(soemMock),
        numberOfSlaves_(numberOfSlaves),
        ioMapSize_(4096),
        failToWriteSdo_(false) {
            ec_slave_.resize(numberOfSlaves_+1);
            ec_group_.resize(1);
            inputs_.resize(numberOfSlaves_);
            outputs_.resize(numberOfSlaves_);
            sdos_.resize(numberOfSlaves_);

            createRegister(0x605A, 0x00);
            createRegister(0x6040, 0x00);

            createRegister(0x1C12, 0x00);
            createRegister(0x1C12, 0x01);
            createRegister(0x1C12, 0x02);

            createRegister(0x1C13, 0x00);
            createRegister(0x1C13, 0x01);
            createRegister(0x1C13, 0x02);
            createRegister(0x1C13, 0x03);
            createRegister(0x1C13, 0x04);
            createRegister(0x1C13, 0x05);

            createRegister(0x6081, 0x00);
            createRegister(0x6083, 0x00);
            createRegister(0x6084, 0x00);
            createRegister(0x6085, 0x00);
            createRegister(0x607F, 0x00);
            createRegister(0x60C5, 0x00);
            createRegister(0x60C6, 0x00);
            createRegister(0x607D, 0x01);
            createRegister(0x607D, 0x02);
            createRegister(0x6075, 0x00);
            createRegister(0x6073, 0x00);
            createRegister(0x6076, 0x00);

            ec_group_[0].outputsWKC = numberOfSlaves_;
            ec_group_[0].inputsWKC = numberOfSlaves_;
            expectedWKC_ = (ec_group_[0].outputsWKC * 2) + ec_group_[0].inputsWKC;
            for (int i=0; i < numberOfSlaves_; i++) {
                inputs_[i].statusWord =
                    crf::devices::ethercatdevices::statusword::IsReadyToSwitchOn_Value;
                outputs_[i].controlWord = 0x0003;
            }

            for (int i=0; i < numberOfSlaves_ + 1; i++) {
                ec_slave_[i].state = EC_STATE_PRE_OP;
            }

            ON_CALL(*soemMock_, init(_)).WillByDefault(Return(1));
            ON_CALL(*soemMock_, config_init(false)).WillByDefault(Return(1));
            ON_CALL(*soemMock_, ec_slavecount()).WillByDefault(Return(numberOfSlaves_));
            ON_CALL(*soemMock_, ec_slave()).WillByDefault(Return(this->ec_slave_.data()));
            ON_CALL(*soemMock_, ec_group()).WillByDefault(Return(this->ec_group_.data()));
            ON_CALL(*soemMock_, readstate()).WillByDefault(Return(1));

            ON_CALL(*soemMock_, config_map(_)).WillByDefault(Invoke([this](void* pIOmap){
                for (int i=0; i < numberOfSlaves_; i++) {
                    ec_slave_[i+1].inputs = reinterpret_cast<uint8*>(&inputs_[i]);
                    ec_slave_[i+1].outputs = reinterpret_cast<uint8*>(&outputs_[i]);
                }
                for (int i=1; i < numberOfSlaves_ + 1; i++) {
                    ec_slave_[i].state = EC_STATE_SAFE_OP;
                }
                return ioMapSize_;
            }));

            ON_CALL(*soemMock_, writestate(_)).WillByDefault(Invoke([this](uint16_t slaveId) {
                if (slaveId == 0) {
                    for (int i=1; i < numberOfSlaves_ + 1; i++) {
                        ec_slave_[i].state = ec_slave_[0].state;
                    }
                }
                return 0;
            }));

            ON_CALL(*soemMock_, readstate()).WillByDefault(
                Invoke([this]() {
                    auto min = ec_slave_[1].state;
                    for (int i=2; i < numberOfSlaves_ + 1; i++) {
                        min = ec_slave_[i].state < min ? ec_slave_[i].state : min;
                    }

                    return min;
                }));

            ON_CALL(*soemMock_, statecheck(_, _, _)).WillByDefault(
                Invoke([this](uint16_t slave, uint16_t reqstate, int timeout) {
                    if (slave == 0) {
                        auto min = ec_slave_[1].state;
                        for (int i=2; i < numberOfSlaves_ + 1; i++) {
                            min = ec_slave_[i].state < min ? ec_slave_[i].state : min;
                        }
                        ec_slave_[0].state = min;
                        return min;
                    } else {
                        return ec_slave_[slave].state;
                    }
                }));

            ON_CALL(*soemMock_, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(
                Invoke([this](uint16_t Slave, uint16_t Index, uint8_t SubIndex, bool CA, int psize, void *p, int Timeout) {  // NOLINT
                    if (failToWriteSdo_) return 0;

                    uint32_t idx = (((uint32_t)Index & 0x00FF) << 16) +
                        ((uint32_t)SubIndex & 0x000F);
                    if (sdos_[Slave-1].find(idx) == sdos_[Slave-1].end()) {
                        std::cout << "Accessing not existing register" << std::endl;
                        return 0;
                    }
                    sdos_[Slave-1][idx] = std::string((char*)p, psize);  // NOLINT
                    return 1;
            }));

            ON_CALL(*soemMock_, SDOread(_, _, _, _, _, _, _)).WillByDefault(
                Invoke([this](uint16_t slave, uint16_t index, uint8_t subindex, bool CA, int *psize, void *p, int timeout) {  // NOLINT
                    auto sdoMap = sdos_[slave-1];
                    uint32_t idx = (((uint32_t)index & 0x00FF) << 16) +
                         ((uint32_t)subindex & 0x000F);
                    if (sdoMap.find(idx) == sdoMap.end()) {
                        std::cout << "Accessing not existing register" << std::endl;
                        return 0;
                    }
                    auto value = sdoMap.at(idx);
                    std::memcpy(p, value.c_str(), *psize);
                    return 1;
            }));

            ON_CALL(*soemMock_, send_processdata()).WillByDefault(Invoke([this](){
                std::unique_lock<std::mutex> lock(pdoMutex_);

                for (int i=0; i < numberOfSlaves_; i++) {
                    inputs_[i].velocityActualValue = outputs_[i].targetVelocity;
                    inputs_[i].positionActualValue = outputs_[i].targetPosition;
                    inputs_[i].torqueActualValue = outputs_[i].targetTorque;
                    inputs_[i].modeOfOperationDisplay = outputs_[i].modeOfOperation;

                    std::bitset<16> statusWordBits(inputs_[i].statusWord);
                    bool inFault = !statusWordBits[6] && statusWordBits[3] && !statusWordBits[2] && !statusWordBits[1] && !statusWordBits[0]; // NOLINT
                    bool switchedOn = !statusWordBits[6] && statusWordBits[5] && !statusWordBits[3] && !statusWordBits[2] && statusWordBits[1] && statusWordBits[0];  // NOLINT
                    bool readyToSwitchOn = !statusWordBits[6] && statusWordBits[5] && !statusWordBits[3] && !statusWordBits[2] && !statusWordBits[1] && statusWordBits[0];  // NOLINT
                    bool enabled = !statusWordBits[6] && statusWordBits[5] && !statusWordBits[3] && statusWordBits[2] && statusWordBits[1] && statusWordBits[0];  // NOLINT
                    bool quickstop = !statusWordBits[6] && !statusWordBits[5] && !statusWordBits[3] && statusWordBits[2] && statusWordBits[1] && statusWordBits[0];  // NOLINT
                    bool switchOnDisabled = statusWordBits[6] && !statusWordBits[3] && !statusWordBits[2] && !statusWordBits[1] && !statusWordBits[0];  // NOLINT

                    std::bitset<16> controlWordBits(outputs_[i].controlWord);

                    if (controlWordBits[7] && inFault) {
                            inputs_[i].statusWord = crf::devices::ethercatdevices::statusword::IsSwitchOnDisabled_Value; // NOLINT
                    } else if (!controlWordBits[7] && controlWordBits[3] && controlWordBits[2] &&
                                controlWordBits[1] && controlWordBits[0] && (quickstop || switchedOn || readyToSwitchOn)) {  // NOLINT
                                    inputs_[i].statusWord =
                                        crf::devices::ethercatdevices::statusword::IsEnabled_Value;
                    } else if (!controlWordBits[7] && !controlWordBits[3] && controlWordBits[2] &&
                                controlWordBits[1] && controlWordBits[0] && (readyToSwitchOn || enabled)) {  // NOLINT
                            inputs_[i].statusWord =
                                crf::devices::ethercatdevices::statusword::IsSwitchedOn_Value;
                    } else if (!controlWordBits[7] && controlWordBits[2] && controlWordBits[1] && !controlWordBits[0] && (switchOnDisabled || enabled || switchedOn)) {  // NOLINT
                            inputs_[i].statusWord =
                                crf::devices::ethercatdevices::statusword::IsReadyToSwitchOn_Value;
                    } else if (!controlWordBits[7] && !controlWordBits[2] && controlWordBits[1] && enabled) {  // NOLINT
                            inputs_[i].statusWord =
                                crf::devices::ethercatdevices::statusword::InQuickStop_Value;
                    } else if (!controlWordBits[7] && !controlWordBits[1] && (enabled || readyToSwitchOn)) {   // NOLINT
                            inputs_[i].statusWord = crf::devices::ethercatdevices::statusword::IsSwitchOnDisabled_Value; // NOLINT
                    }

                    if (!controlWordBits[7] && controlWordBits[4]) {
                            inputs_[i].statusWord = inputs_[i].statusWord |
                            crf::devices::ethercatdevices::statusword::bit_setNewPointAck;
                    } else if (!controlWordBits[7] && controlWordBits[8]) {
                            inputs_[i].statusWord = inputs_[i].statusWord |
                            crf::devices::ethercatdevices::statusword::bit_targetReached;
                    }
                }

                pdoExchanged_ = true;
                pdoCv_.notify_all();
                return 0;
            }));

            ON_CALL(*soemMock_, receive_processdata(_)).WillByDefault(Return(expectedWKC_));
    }

    bool waitForProcessData() {
        std::unique_lock<std::mutex> lock(pdoMutex_);
        pdoExchanged_ = false;
        return pdoCv_.wait_for(lock, std::chrono::milliseconds(200), [this]() {
            return pdoExchanged_;
        });
    }

    std::shared_ptr<SoemApiMock> soemMock_;
    uint8_t numberOfSlaves_;
    int expectedWKC_;

    std::vector<EC_input> inputs_;
    std::vector<EC_output> outputs_;

    std::mutex pdoMutex_;
    std::condition_variable pdoCv_;
    bool pdoExchanged_;

    int ioMapSize_;
    std::vector<ec_slavet> ec_slave_;
    std::vector<ec_groupt> ec_group_;

    std::vector<std::map<uint32_t, std::string> > sdos_;

    bool failToWriteSdo_;

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
