/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <regex>
#include <string>
#include <fstream>
#include <vector>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "UniversalRobot/UniversalRobotInterfaceMock.hpp"
#include "UniversalRobot/UniversalRobotRTDE.hpp"
#include "UniversalRobot/UniversalRobotRTDEInterface.hpp"
#include "UniversalRobot/IUniversalRobotRTDEInterface.hpp"

using testing::_;
using testing::AnyNumber;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::SaveArg;
using testing::Return;

using crf::actuators::universalrobot::UniversalRobotRTDE;
using crf::actuators::universalrobot::UniversalRobotInterfaceMock;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointForceTorques;

class UniversalRobotShould: public ::testing::Test {
 protected:
    UniversalRobotShould():
        logger_("UniversalRobotShould") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("UniversalRobotDeprecated"));
        std::string configFilePath(testDirName +
            "UniversalRobotDeprecated/tests/config/UR10e.json");
        logger_->debug(configFilePath);
        std::ifstream configIfStream(configFilePath);
        robotConfigFile_ = nlohmann::json::parse(configIfStream);
        UniversalRobotInterfaceMock_.reset(new NiceMock<UniversalRobotInterfaceMock>);
    }

    ~UniversalRobotShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        ON_CALL(*UniversalRobotInterfaceMock_, getActualQ()).
            WillByDefault(Invoke(returnSimulatedJointPositions));
        ON_CALL(*UniversalRobotInterfaceMock_, getActualQd()).
            WillByDefault(Invoke(returnSimulatedJointVelocities));
        ON_CALL(*UniversalRobotInterfaceMock_, getActualTCPPose()).
            WillByDefault(Invoke(returnSimulatedTaskPose));
        ON_CALL(*UniversalRobotInterfaceMock_, servoJ(_, _, _, _, _, _)).
            WillByDefault(Invoke(returnSimulatedServoJResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, speedJ(_, _, _)).
            WillByDefault(Invoke(returnSimulatedSpeedJResponse));

        sut_.reset(new UniversalRobotRTDE(UniversalRobotInterfaceMock_, robotConfigFile_));
    }

    static std::vector<double> returnSimulatedJointPositions() {
        std::vector<double> simJointPositions{0, 0, 0, 0, 0, 0};
        return simJointPositions;
    }

    static std::vector<double> returnSimulatedJointVelocities() {
        std::vector<double> simJointVelocities{0, 0, 0, 0, 0, 0};
        return simJointVelocities;
    }

    static std::vector<double> returnSimulatedTaskPose() {
        std::vector<double> simTaskPose{0, 0, 0, 0, 0, 0};
        return simTaskPose;
    }

    static bool returnSimulatedServoJResponse(std::vector<double> q, double v, double a,
        double cycleTime, double lookAheadTime, double gain) {
        // Using double cast to compensate error from IEEE754 with b=2
        if (!(cycleTime >= 0.002 &&
            lookAheadTime >= static_cast<double>(static_cast<float>(0.03)) &&
            lookAheadTime <= 0.2 && gain >= 100 && gain <= 2000)) {
            return false;
        }
        return true;
    }

    static bool returnSimulatedSpeedJResponse(std::vector<double> qd, double maxAcc,
        double timeUntilFunctionReturns) {
        if (!(timeUntilFunctionReturns > 0 && maxAcc >= 0 && maxAcc <= 40)) {
            return false;
        }
        return true;
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<UniversalRobotInterfaceMock>> UniversalRobotInterfaceMock_;
    nlohmann::json robotConfigFile_;
    std::unique_ptr<UniversalRobotRTDE> sut_;
};

TEST_F(UniversalRobotShould, returnFalseIfInitializedMultipleTimes) {
    EXPECT_CALL(*UniversalRobotInterfaceMock_, initRtdeReceiveInterface(_)).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(UniversalRobotShould, returnFalseIfDeinitializedMultipleTimes) {
    EXPECT_CALL(*UniversalRobotInterfaceMock_, stopScript()).Times(1);
    EXPECT_CALL(*UniversalRobotInterfaceMock_, speedStop()).Times(1).WillOnce(Return(true));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

// Test behavior for init with wrong ip adress not necesssary, because they are fixed.
TEST_F(UniversalRobotShould, returnValidJointPositionsIfInitialized) {
    JointPositions q({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions().get(), q, 10e-5));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnValidJointVelocitiesIfInitialized) {
    JointVelocities qd({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointVelocities().get(), qd, 10e-5));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnValidTaskPosesIfInitialized) {
    TaskPose z({0, 0, 0}, crf::math::rotation::CardanXYZ({ 0, 0, 0}));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskPose().get(), z, 10e-5));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfFeasibleSetJointPositionsInput) {
    std::vector<JointPositions>q{JointPositions({0, 0, 0, 0, 0, 0}),
        JointPositions({1, 2, 3, 4, 5, 6}),
        JointPositions({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointPositions(q[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseIfInfeasibleSetJointPositionsInput) {
    std::vector<JointPositions> q{JointPositions({7, 0, 0, 0, 0, 0}),
        JointPositions({-7, 0, 0, 0, 0, 0}),
        JointPositions({1, 2, 3, 4, 5, 6, 0}),
        JointPositions({1, 2, 3, 4, 5}),
        JointPositions({-1.1, 2.5, -3.6, 4.12345, -7.35682, 6})};

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < q.size(); i++) {
        ASSERT_FALSE(sut_->setJointPositions(q[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseIfSetJointVelocitiesCalledWithoutAcceleration) {
    JointVelocities qd({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setJointVelocities(qd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfFeasibleSetJointVelocitiesInput) {
    std::vector<JointVelocities> qd{JointVelocities({0, 0, 0, 0, 0, 0}),
        JointVelocities({1, 2, 3, 3, 3, 3}),
        JointVelocities({-1.1, -2.0, -2.8, 1.23453, -2.1234, 0})};
    std::vector<double> qddLead{1, 10, 20.23};

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qd.size(); i++) {
        ASSERT_TRUE(sut_->setJointVelocities(qd[i], qddLead[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseIfInfeasibleSetJointVelocitiesInput) {
    std::vector<JointVelocities> qd{JointVelocities({3.1, 0, 0, 0, 0, 0}),
        JointVelocities({-3.1, 0, 0, 0, 0, 0}),
        JointVelocities({0, 4.1, 0, 0, 0, 0}),
        JointVelocities({0, -4.3, 0, 0, 0, 0}),
        JointVelocities({0, 0, 0, 3.5, 0, 0}),
        JointVelocities({0, 0, 0, 0, 0, -4.7}),
        JointVelocities({1, 2, 3, 3, 3, 3, 0}),
        JointVelocities({1, 2, 3, 3, 3}),
        JointVelocities({0, 0, 0, 0, 0, 0}),
        JointVelocities({0, 0, 0, 0, 0, 0})};
    std::vector<double> qddLead{1, 10, 20.23, 1, 1, 1, 1, 1, -1, 50};

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qd.size(); i++) {
        ASSERT_FALSE(sut_->setJointVelocities(qd[i], qddLead[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseIfNotInitialized) {
    ASSERT_FALSE(sut_->stopArm());
}

TEST_F(UniversalRobotShould, returnNoneIfNotInitialized) {
    ASSERT_EQ(sut_->getJointVelocities(), boost::none);
    ASSERT_EQ(sut_->getJointPositions(), boost::none);
}

TEST_F(UniversalRobotShould, returnFalseIfMethodNotImplemented) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setTaskPose(
        TaskPose({1, 2, 3},
        crf::math::rotation::CardanXYZ({ 4, 5, 6}))));
    ASSERT_FALSE(sut_->setJointForceTorques(JointForceTorques({1, 2, 3, 4, 5, 6})));
    ASSERT_FALSE(sut_->enableBrakes());
    ASSERT_FALSE(sut_->disableBrakes());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnNoneIfMethodNotImplemented) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(sut_->getTaskForceTorque(), boost::none);
    ASSERT_EQ(sut_->getTaskVelocity(), boost::none);
    ASSERT_EQ(sut_->getJointForceTorques(), boost::none);
    ASSERT_TRUE(sut_->deinitialize());
}
