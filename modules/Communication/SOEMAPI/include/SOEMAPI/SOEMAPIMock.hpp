/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN EN/SMM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "ISOEMAPI.hpp"

namespace crf::communication::soemapi {

class SOEMAPIMock : public ISOEMAPI {
 public:
    MOCK_METHOD(int, init, (char* ifname), (override));
    MOCK_METHOD(int, config_init, (uint8_t usetable), (override));
    MOCK_METHOD(int, ec_slavecount, (), (override));
    MOCK_METHOD(ec_slavet*, ec_slave, (), (override));
    MOCK_METHOD(ec_groupt*, ec_group, (), (override));
    MOCK_METHOD(int, config_overlap_map, (void* pIOmap), (override));
    MOCK_METHOD(int, send_overlap_processdata, (), (override));
    MOCK_METHOD(int, receive_processdata, (int timeout), (override));
    MOCK_METHOD(int, SDOread, (uint16_t slave, uint16_t index, uint8_t subindex, bool CA, int *psize, void *p, int timeout), (override));  // NOLINT
    MOCK_METHOD(int, SDOwrite, (uint16_t slave, uint16_t index, uint8_t subindex, bool CA, int psize, void *p, int timeout), (override));  // NOLINT
    MOCK_METHOD(uint16_t, statecheck, (uint16_t slave, uint16_t reqstate, int timeout), (override));
    MOCK_METHOD(int, writestate, (uint16_t slave), (override));
    MOCK_METHOD(int, readstate, (), (override));
    MOCK_METHOD(void, close, (), (override));
    MOCK_METHOD(int, reconfig_slave, (uint16_t slave, int timeout), (override));
    MOCK_METHOD(int, recover_slave, (uint16_t slave, int timeout), (override));
    MOCK_METHOD(bool, configdc, (), (override));
};

}  // namespace crf::communication::soemapi
