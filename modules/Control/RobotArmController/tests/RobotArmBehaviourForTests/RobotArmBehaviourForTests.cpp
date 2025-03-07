/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 * 
 *  ==================================================================================================
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "RobotArmBehaviourForTests.hpp"

RobotArmBehaviourForTests::RobotArmBehaviourForTests():
    logger_("RobotArmBehaviourForTests"),
    simulatedJointPositions_(NUM_JOINTS),
    simulatedJointVelocities_(NUM_JOINTS) {
    logger_->info("{} BEGIN",
        testing::UnitTest::GetInstance()->current_test_info()->name());
    testDirName_ = __FILE__;
    testDirName_ = testDirName_.substr(0,
        testDirName_.find(
            "RobotArmBehaviourForTests/RobotArmBehaviourForTests.cpp"));
    robotArmMock_.reset(new NiceMock<RobotArmMock>);

    robotArmConfiguration_.reset(new RobotArmConfiguration);
    std::ifstream robotData(testDirName_ + "/config/KinovaConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    robotArmConfiguration_->parse(robotJSON);
    kinematics_.reset(new RobotArmKDLKinematics(robotArmConfiguration_));

    std::vector<double> JointKp = robotArmConfiguration_->getParametersPIDs().KpJoint;
    std::vector<double> JointKi = robotArmConfiguration_->getParametersPIDs().KiJoint;
    std::vector<double> JointKd = robotArmConfiguration_->getParametersPIDs().KdJoint;

    std::vector<double> TaskKp = robotArmConfiguration_->getParametersPIDs().KpTask;
    std::vector<double> TaskKi = robotArmConfiguration_->getParametersPIDs().KiTask;
    std::vector<double> TaskKd = robotArmConfiguration_->getParametersPIDs().KdTask;

    jointPIDController_ =
        std::make_shared<crf::control::closedloopcontroller::PIDController>(
            JointKp,
            JointKi,
            JointKd);
    taskPIDController_ =
        std::make_shared<crf::control::closedloopcontroller::PIDController>(
            TaskKp,
            TaskKi,
            TaskKd);

    lastSetVelocityTime_ = std::chrono::high_resolution_clock::now();
}

RobotArmBehaviourForTests::~RobotArmBehaviourForTests() {
    logger_->info("{} END with {}",
        testing::UnitTest::GetInstance()->current_test_info()->name(),
        testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
}

void RobotArmBehaviourForTests::SetUp() {
    configureRobotArmDefaultBehavior();
}

void RobotArmBehaviourForTests::configureRobotArmDefaultBehavior() {
    ON_CALL(*robotArmMock_, getConfiguration()).WillByDefault(Return(robotArmConfiguration_));

    ON_CALL(*robotArmMock_, setJointVelocities(_)).WillByDefault(Invoke(
        [this](const JointVelocities& jointVelocities) {
            auto start = std::chrono::high_resolution_clock::now();

            float timestep = static_cast<float>(std::chrono::duration_cast
                <std::chrono::microseconds>(start - lastSetVelocityTime_).count())/1e6;

            for (int i = 0; i < NUM_JOINTS; i++) {
                simulatedJointPositions_[i] += jointVelocities[i]*timestep;
            }
            simulatedJointVelocities_ = jointVelocities;
            lastSetVelocityTime_ = start;
            return true;
        }));
    ON_CALL(*robotArmMock_, getJointPositions())
        .WillByDefault(Invoke([this](){ return simulatedJointPositions_;}));
    ON_CALL(*robotArmMock_, getJointVelocities())
        .WillByDefault(Invoke([this](){ return simulatedJointVelocities_;}));
    ON_CALL(*robotArmMock_, stopArm())
        .WillByDefault(Return(true));
    ON_CALL(*robotArmMock_, enableBrakes())
        .WillByDefault(Return(true));
    ON_CALL(*robotArmMock_, disableBrakes())
        .WillByDefault(Return(true));
}
