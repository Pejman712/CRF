/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 * 
 *  ==================================================================================================
 */
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "RobotArm/RobotArmConfiguration.hpp"
#include "RobotArm/RobotArmMock.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"
#include "ClosedLoopController/PIDController.hpp"

#include "Types/Types.hpp"
#include "EventLogger/EventLogger.hpp"

#define NUM_JOINTS 6

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointForceTorques;
using crf::utility::types::areAlmostEqual;

using crf::actuators::robotarm::RobotArmConfiguration;
using crf::control::robotarmkinematics::RobotArmKDLKinematics;
using crf::actuators::robotarm::RobotArmMock;

class RobotArmBehaviourForTests: public ::testing::Test {
 protected:
    RobotArmBehaviourForTests();
    ~RobotArmBehaviourForTests();

    void SetUp() override;
    void configureRobotArmDefaultBehavior();

    std::shared_ptr<RobotArmKDLKinematics> kinematics_;
    std::shared_ptr<NiceMock<RobotArmMock>> robotArmMock_;

    std::shared_ptr<crf::control::closedloopcontroller::PIDController> jointPIDController_;
    std::shared_ptr<crf::control::closedloopcontroller::PIDController> taskPIDController_;
    std::shared_ptr<RobotArmConfiguration> robotArmConfiguration_;

 private:
    std::chrono::time_point<std::chrono::high_resolution_clock> lastSetVelocityTime_;
    crf::utility::logger::EventLogger logger_;
    JointPositions simulatedJointPositions_;
    JointVelocities simulatedJointVelocities_;
    std::string testDirName_;
};
