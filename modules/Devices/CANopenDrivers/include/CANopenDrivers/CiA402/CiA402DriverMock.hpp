/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *         Sebastien Collomb CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>

#include "crf/expected.hpp"

#include "CANopenDrivers/CiA402/ICiA402Driver.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace crf::devices::canopendrivers {

class CiA402DriverMock: public ICiA402Driver {
 public:
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(bool, deinitialize, (), (override));

    MOCK_METHOD(bool, inFault, (), (override));
    MOCK_METHOD(bool, inQuickStop, (), (override));
    MOCK_METHOD(bool, resetFault, (), (override));
    MOCK_METHOD(bool, quickStop, (), (override));
    MOCK_METHOD(void, stop, (), (override));
    MOCK_METHOD(bool, resetQuickStop, (), (override));

    MOCK_METHOD(crf::expected<bool>, setModeOfOperation, (const ModeOfOperation&), (override));
    MOCK_METHOD(crf::expected<bool>, setMaximumTorque, (double maximumTorque), (override));
    MOCK_METHOD(crf::expected<double>, getPosition, (), (override));
    MOCK_METHOD(crf::expected<double>, getVelocity, (), (override));
    MOCK_METHOD(crf::expected<double>, getTorque, (), (override));
    MOCK_METHOD(crf::expected<double>, getMaximumTorque, (), (override));
    MOCK_METHOD(ModeOfOperation, getModeOfOperation, (), (override));
    MOCK_METHOD(std::vector<crf::ResponseCode>, getMotorStatus, (), (override));
    MOCK_METHOD(StatusWord, getStatusWord, (), (override));

    MOCK_METHOD(crf::expected<bool>, setProfilePosition,
        (double pos, double vel, double acc, double dec, PositionReference reference), (override));
    MOCK_METHOD(crf::expected<bool>, setVelocity, (double vel, double deltaSpeedAcc,
        double deltaTimeAcc, double deltaSpeedDec, double delatTimeDec), (override));
    MOCK_METHOD(crf::expected<bool>, setProfileVelocity,
        (double vel, double acc, double dec), (override));
    MOCK_METHOD(crf::expected<bool>, setProfileTorque, (double tor), (override));
    MOCK_METHOD(crf::expected<bool>, setInterpolatedPosition,
        (double pos, double vel, double acc, double dec), (override));
    MOCK_METHOD(crf::expected<bool>, setCyclicPosition,
        (double pos, double posOffset, double velOffset, double torOffset), (override));
    MOCK_METHOD(crf::expected<bool>, setCyclicVelocity,
        (double vel, double velOffset, double torOffset), (override));
    MOCK_METHOD(crf::expected<bool>, setCyclicTorque, (double tor, double torOffset), (override));
};

}  // namespace crf::devices::canopendrivers
