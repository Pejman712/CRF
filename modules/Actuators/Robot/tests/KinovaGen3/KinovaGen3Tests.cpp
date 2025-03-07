/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
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
#include "KortexAPI/KortexMovementAPIInterfaceMock.hpp"
#include "KortexAPI/KortexMovementAPIInterfaceMockConfiguration.hpp"
#include "Robot/KinovaGen3/KinovaGen3.hpp"
#include "KortexAPI/KortexMovementAPIInterface.hpp"
#include "KortexAPI/IKortexMovementAPIInterface.hpp"

using testing::_;
using testing::AnyNumber;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::SaveArg;
using testing::Return;

using crf::actuators::robot::KinovaGen3;
using crf::communication::kortexapi::KortexMovementAPIInterfaceMockConfiguration;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

using crf::actuators::robot::KinovaGen3Configuration;

class KinovaGen3Should: public ::testing::Test {
 protected:
    KinovaGen3Should():
    logger_("KinovaGen3Should") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("KinovaGen3Tests.cpp"));
        std::string configFilePath(testDirName + "config/KinovaGen3.json");
        logger_->debug(configFilePath);
        std::ifstream configIfStream(configFilePath);
        robotConfigFile_ = nlohmann::json::parse(configIfStream);
    }

    ~KinovaGen3Should() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        KortexAPIInterfaceMock_.reset(new NiceMock<KortexMovementAPIInterfaceMockConfiguration>);
        KortexAPIInterfaceMock_->configureMock();
        sut_.reset(new KinovaGen3(
            KortexAPIInterfaceMock_, KinovaGen3Configuration(robotConfigFile_)));
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<KortexMovementAPIInterfaceMockConfiguration>> KortexAPIInterfaceMock_;
    nlohmann::json robotConfigFile_;
    std::unique_ptr<KinovaGen3> sut_;
};

TEST_F(KinovaGen3Should, returnTrueIfInitializedMultipleTimes) {
    EXPECT_CALL(*KortexAPIInterfaceMock_, connect(_, _, _)).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(KinovaGen3Should, returnTrueIfDeinitializedMultipleTimes) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnValidJointPositionsIfInitialized) {
    JointPositions q({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions().value(), q));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnFalseJointPositionsIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointPositions());
}

TEST_F(KinovaGen3Should, returnValidJointVelocitiesIfInitialized) {
    JointVelocities qd({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointVelocities().value(), qd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnFalseJointVelocitiesIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointVelocities());
}

TEST_F(KinovaGen3Should, returnFalseOnJointAccelerationsNotDefined) {
    ASSERT_FALSE(sut_->getJointAccelerations());
}

TEST_F(KinovaGen3Should, returnValidJointForceTorquesIfInitialized) {
    JointForceTorques jT({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointForceTorques().value(), jT));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnValidJointForceTorquesIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointForceTorques());
}

TEST_F(KinovaGen3Should, returnValidTaskPosesIfInitialized) {
    TaskPose z(Eigen::Vector3d({0, 0, 0}), crf::math::rotation::CardanXYZ({0, 0, 0}));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskPose().value(), z));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnValidTaskPosesIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskPose());
}

TEST_F(KinovaGen3Should, returnValidTaskVelocityIfInitialized) {
    std::vector<double> tV{0, 0, 0, 0, 0, 0};
    TaskVelocity zd({tV[0], tV[1], tV[2], tV[3], tV[4], tV[5]});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskVelocity().value(), zd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnValidTaskVelocityIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskVelocity());
}

TEST_F(KinovaGen3Should, returnValidTaskAccelerationIfNotDefined) {
    ASSERT_FALSE(sut_->getTaskAcceleration());
}

TEST_F(KinovaGen3Should, returnValidTaskWrenchIfInitialized) {
    std::vector<double> tT{0, 0, 0, 0, 0, 0};
    TaskForceTorque t({tT[0], tT[1], tT[2], tT[3], tT[4], tT[5]});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskForceTorque().value(), t));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnFalseTaskWrenchIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskForceTorque());
}

TEST_F(KinovaGen3Should, returnTrueIfFeasibleSetJointPositionsInput) {
    std::vector<JointPositions>q{JointPositions({0, 0, 0, 0, 0, 0}),
        JointPositions({6.2, 2.2, 2.5, 6.2, 2.0, 6.2}),
        JointPositions({1, 2, 3, 4, 5, 6})};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setJointPositions(true, q[0]));
    ASSERT_TRUE(sut_->setJointPositions(true, q[1]));
    ASSERT_FALSE(sut_->setJointPositions(true, q[2]));
    ASSERT_TRUE(sut_->setJointPositions(false, q[0]));
    ASSERT_TRUE(sut_->setJointPositions(false, q[1]));
    ASSERT_FALSE(sut_->setJointPositions(false, q[2]));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnFalseIfInfeasibleSetJointPositionsInput) {
    std::vector<JointPositions> q{JointPositions({7, 0, 0, 0, 0, 0}),
        JointPositions({-7, 0, 0, 0, 0, 0}),
        JointPositions({1, 2, 3, 4, 5, 6, 0}),
        JointPositions({1, 2, 3, 4, 5}),
        JointPositions({-1.1, 2.5, -3.6, 4.12345, -7.35682, 6})};

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < q.size(); i++) {
        ASSERT_FALSE(sut_->setJointPositions(true, q[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnTrueIfSetJointVelocitiesCalledWithoutAcceleration) {
    JointVelocities qd({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setJointVelocities(true, qd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, returnFalseIfInfeasibleSetJointVelocitiesInput) {
    std::vector<JointVelocities> qd{JointVelocities({0, 400, 0, 0, 0, 0}),
        JointVelocities({0, -400, 0, 0, 0, 0}),
        JointVelocities({1, 2, 3, 3, 3, 3, 0}),
        JointVelocities({1, 2, 3, 3, 3})};
    JointAccelerations qddLead({1, 10, 20.23, 1, 1, 1});
    JointVelocities qdGood({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qd.size(); i++) {
        ASSERT_FALSE(sut_->setJointVelocities(true, qd[i], qddLead));
        ASSERT_FALSE(sut_->setJointVelocities(true, qd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(KinovaGen3Should, throwCorrespondentErrorCodeWhenCallingInvalidFunctions) {
    JointForceTorques jt({1, 1, 1, 1, 1, 1});
    TaskPose tp(Eigen::Vector3d({1, 1, 1}),
        crf::math::rotation::CardanXYZ({1, 1, 1}));
    TaskVelocity tv({0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    TaskForceTorque tt({0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    TaskAcceleration ta({0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    JointVelocities jv({1, 1, 1, 1, 1, 1});
    JointAccelerations ja({0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    std::vector<bool> status = {true};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->getJointVelocities());
    ASSERT_FALSE(sut_->getTaskAcceleration());
    ASSERT_FALSE(sut_->getJointAccelerations());
    ASSERT_FALSE(sut_->setJointForceTorques(true, jt));
    ASSERT_FALSE(sut_->setTaskPose(true, tp));
    ASSERT_FALSE(sut_->setTaskVelocity(true, tv));
    ASSERT_FALSE(sut_->setTaskForceTorque(true, tt));
    ASSERT_FALSE(sut_->getProfileJointVelocities());
    ASSERT_FALSE(sut_->getProfileJointAccelerations());
    ASSERT_FALSE(sut_->getProfileTaskVelocity());
    ASSERT_FALSE(sut_->getProfileTaskAcceleration());
    ASSERT_FALSE(sut_->setProfileJointVelocities(jv));
    ASSERT_FALSE(sut_->setProfileJointAccelerations(ja));
    ASSERT_FALSE(sut_->setProfileTaskVelocity(tv));
    ASSERT_FALSE(sut_->setProfileTaskAcceleration(ta));
    ASSERT_FALSE(sut_->softStop());
    ASSERT_FALSE(sut_->hardStop());
    ASSERT_FALSE(sut_->setBrakes(status));
}
