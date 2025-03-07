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

#include "UniversalRobot/IUniversalRobotRTDEInterface.hpp"

namespace crf::actuators::universalrobot {

class UniversalRobotInterfaceMock : public IUniversalRobotRTDEInterface {
 public:
    MOCK_METHOD(void, initRtdeReceiveInterface, (std::string IP), (override));
    MOCK_METHOD(std::vector<double>, getActualQ, (), (override));
    MOCK_METHOD(std::vector<double>, getActualQd, (), (override));
    MOCK_METHOD(std::vector<double>, getActualTCPPose, (), (override));
    MOCK_METHOD(void, initRtdeControlInterface, (std::string IP), (override));
    MOCK_METHOD(bool, servoJ, (std::vector<double> qDes, double maxVel, double maxAcc,
        double loopTime, double lookAheadTime, double gain), (override));
    MOCK_METHOD(bool, speedJ, (std::vector<double> qdDes, double maxAcceleration, double loopTime),
        (override));
    MOCK_METHOD(bool, speedStop, (), (override));
    MOCK_METHOD(bool, servoStop, (), (override));
    MOCK_METHOD(void, stopScript, (), (override));
};

}  // namespace crf::actuators::universalrobot
