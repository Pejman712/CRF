/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <vector>
#include <string>

#include "UniversalRobotRTDE/IUniversalRobotRTDEInterface.hpp"

namespace crf::communication::universalrobotrtde {

class UniversalRobotRTDEInterfaceMock : public IUniversalRobotRTDEInterface {
 public:
    MOCK_METHOD(void, initRtdeReceiveInterface, (std::string IP), (override));
    MOCK_METHOD(std::vector<double>, getActualQ, (), (override));
    MOCK_METHOD(std::vector<double>, getActualQd, (), (override));
    MOCK_METHOD(std::vector<double>, getActualTCPPose, (), (override));
    MOCK_METHOD(std::vector<double>, getActualTCPSpeed, (), (override));
    MOCK_METHOD(std::vector<double>, getActualTCPForce, (), (override));
    MOCK_METHOD(uint32_t, getRobotStatus, (), (override));
    MOCK_METHOD(int32_t, getRobotMode, (), (override));
    MOCK_METHOD(uint32_t, getSafetyStatusBits, (), (override));
    MOCK_METHOD(void, initRtdeControlInterface, (std::string IP), (override));
    MOCK_METHOD(bool, servoJ, (std::vector<double> qDes, double maxVel, double maxAcc,
        double loopTime, double lookAheadTime, double gain), (override));
    MOCK_METHOD(bool, speedJ, (std::vector<double> qdDes, double maxAcceleration, double loopTime),
        (override));
    MOCK_METHOD(bool, speedL, (std::vector<double> zdDes, double maxAcceleration, double loopTime),
        (override));
    MOCK_METHOD(bool, moveJ, (std::vector<double> qDes, double speed, double acceleration,
        bool async), (override));
    MOCK_METHOD(bool, moveL, (std::vector<double> zDes, double speed, double acceleration,
        bool async), (override));
    MOCK_METHOD(bool, setGravity, (const std::vector<double>& gravity), (override));
    MOCK_METHOD(bool, zeroFtSensor, (), (override));
    MOCK_METHOD(bool, forceMode, (std::vector<double> forceFrame,
        std::vector<int> complianceSelector, std::vector<double> desiredForceTorque, int type,
        std::vector<double> limits), (override));
    MOCK_METHOD(std::vector<double>, getJointForceTorques, (), (override));
    MOCK_METHOD(bool, speedStop, (), (override));
    MOCK_METHOD(bool, servoStop, (), (override));
    MOCK_METHOD(void, stopScript, (), (override));
};

}  // namespace crf::communication::universalrobotrtde
