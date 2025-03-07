/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *         Jorge Playan Garai BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

#include <atomic>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "SOEMAPIMock.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

namespace crf::communication::soemapi {

class SOEMAPIMockConfiguration : public SOEMAPIMock {
 public:
    SOEMAPIMockConfiguration() = delete;
    SOEMAPIMockConfiguration(const uint8_t& numberOfSlaves, const int& ioMapSize):
        numberOfSlaves_(numberOfSlaves),
        ioMapSize_(ioMapSize),
        slaves_(numberOfSlaves_ + 1),
        slaveGroups_(1),
        stateForced_(false) {
            slaveGroups_[0].outputsWKC = numberOfSlaves_;
            slaveGroups_[0].inputsWKC = numberOfSlaves_;

            expectedWKC_ = (slaveGroups_[0].outputsWKC * 2) + slaveGroups_[0].inputsWKC;
    }

    const uint16_t ALL_SLAVES = 0;

    void forceSlaveState(uint16_t slave, uint16_t reqstate) {
        stateForced_ = true;
        slaves_[slave].state = reqstate;
    }

    virtual void configure() {
        ON_CALL(*this, ec_slavecount()).WillByDefault(Return(numberOfSlaves_));
        ON_CALL(*this, ec_slave()).WillByDefault(Return(slaves_.data()));
        ON_CALL(*this, ec_group()).WillByDefault(Return(slaveGroups_.data()));
        ON_CALL(*this, readstate()).WillByDefault(Return(1));
        ON_CALL(*this, init(_)).WillByDefault(Return(1));

        ON_CALL(*this, config_init(_)).WillByDefault(Invoke(
            [this](bool smth) {
                if (stateForced_) return 1;
                // Move slaves to PRE-OPERATIONAL
                for (int i = 0; i < numberOfSlaves_ + 1; i++) {
                    slaves_[i].state = EC_STATE_PRE_OP;
                }
                return 1;
            }));

        ON_CALL(*this, config_overlap_map(_)).WillByDefault(Invoke(
            [this](void* pIOmap) {
                if (stateForced_) return ioMapSize_;
                // Move slaves to SAFE-OPERATIONAL
                for (int i = 0; i < numberOfSlaves_ + 1; i++) {
                    slaves_[i].state = EC_STATE_SAFE_OP;
                }
                return ioMapSize_;
            }));

        ON_CALL(*this, writestate(_)).WillByDefault(Invoke(
            [this] (uint16_t slaveId) {
                if (stateForced_) {
                    for (int i = 0; i < numberOfSlaves_ + 1; i++) {
                        slaves_[i].state = state_;
                    }
                    return 0;
                }
                if (slaveId != ALL_SLAVES) {
                    slaves_[slaveId].state = slaves_[ALL_SLAVES].state;
                    return 0;
                }
                for (int i = 1; i < numberOfSlaves_ + 1; i++) {
                    slaves_[i].state = slaves_[ALL_SLAVES].state;
                }
                return 0;
            }));

        ON_CALL(*this, readstate()).WillByDefault(Invoke(
            [this] () {
                uint16_t min = slaves_[1].state;
                for (int i = 2; i < numberOfSlaves_ + 1; i++) {
                    if (min > slaves_[i].state) min = slaves_[i].state;
                }
                return min;
            }));

        ON_CALL(*this, statecheck(_, _, _)).WillByDefault(Invoke(
            [this](uint16_t slave, uint16_t reqstate, int timeout) {
                if (stateForced_) return slaves_[slave].state;

                if (slave != ALL_SLAVES) {
                    slaves_[slave].state = reqstate;
                    return reqstate;
                }
                for (int i = 0; i < numberOfSlaves_ + 1; i++) {
                    slaves_[i].state = reqstate;
                }
                return reqstate;
            }));

        ON_CALL(*this, send_overlap_processdata()).WillByDefault(Return(0));
        ON_CALL(*this, receive_processdata(_)).WillByDefault(Return(expectedWKC_));
    }

 protected:
    uint8_t numberOfSlaves_;
    int ioMapSize_;

    std::vector<ec_slavet> slaves_;
    std::vector<ec_groupt> slaveGroups_;

    int expectedWKC_;

    std::atomic<bool> stateForced_;
    std::atomic<uint16_t> state_;
};

}  // namespace crf::communication::soemapi
