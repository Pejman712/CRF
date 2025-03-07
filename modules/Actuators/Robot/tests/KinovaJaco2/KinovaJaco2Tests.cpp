/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <KinovaTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <kdl/frames.hpp>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>

#include "Robot/IRobot.hpp"
#include "KinovaJacoAPI/KinovaJacoAPIInterfaceMockConfiguration.hpp"
#include "Robot/KinovaJaco2/KinovaJaco2.hpp"

#include "EventLogger/EventLogger.hpp"

#define KINOVA_JACO_NUM_JOINTS 6

using crf::communication::kinovajacoapi::KinovaJacoAPIInterfaceMockConfiguration;
using crf::actuators::robot::KinovaJaco2;
using crf::actuators::robot::KinovaJaco2Configuration;

using testing::_;
using testing::Invoke;
using testing::NiceMock;

class KinovaJaco2Should: public ::testing::Test {
 protected:
    KinovaJaco2Should():
        logger_("KinovaJaco2Should") {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());

            testDirName_ = __FILE__;
            testDirName_ = testDirName_.substr(0, testDirName_.find("KinovaJaco2Tests.cpp"));
            testDirName_ += "config/";
    }
    ~KinovaJaco2Should() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        apiInterfaceMock_.reset(new NiceMock<KinovaJacoAPIInterfaceMockConfiguration>);
        apiInterfaceMock_->configureMock();
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<KinovaJacoAPIInterfaceMockConfiguration>> apiInterfaceMock_;
    std::unique_ptr<KinovaJaco2> sut_;
    std::string testDirName_;
};

TEST_F(KinovaJaco2Should, returnFalseIfInitializedOrDeinitializedTwice) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco2(
        apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
        sut_->initialize();
        sut_->initialize();
        sut_->deinitialize();
        sut_->deinitialize();
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(KinovaJaco2Should, returnFalseIfInitializedOrDeinitializedTwiceURDF) {
    std::ifstream robotData(testDirName_ + "goodConfigurationURDFWithTool.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new KinovaJaco2(
        apiInterfaceMock_, KinovaJaco2Configuration(robotJSON)));
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco2(
        apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(KinovaJaco2Should, ThrowWhenIncorrectConfigFileProvided) {
    std::ifstream robotData_badKey(testDirName_ + "configFile_badKey.json");
    nlohmann::json robotJSON_badKey = nlohmann::json::parse(robotData_badKey);
    ASSERT_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON_badKey))),
        std::invalid_argument);
}

TEST_F(KinovaJaco2Should, returnEmptyVectorWhenJointsGetterInvokedAndNotInitialized) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_FALSE(sut_->getJointPositions());
    ASSERT_FALSE(sut_->getJointVelocities());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(KINOVA_JACO_NUM_JOINTS, sut_->getJointPositions().value().size());
    ASSERT_EQ(KINOVA_JACO_NUM_JOINTS, sut_->getJointVelocities().value().size());
}

TEST_F(KinovaJaco2Should, returnEmptyMatrixWhenArmPositionGetterInvokeAndNotInitialized) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_FALSE(sut_->getTaskPose());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->getTaskPose());
}

TEST_F(KinovaJaco2Should, returnFalseWhenStopperInvokeAndNotInitialized) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_FALSE(sut_->softStop());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->softStop());
}

TEST_F(KinovaJaco2Should, returnJointPositionsIfApiCallSuccessful) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_TRUE(sut_->initialize());
    JointPositions expectedJointPositions({-M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, -M_PI/2});
    auto result = sut_->getJointPositions().value();
    for (int i=0; i < KINOVA_JACO_NUM_JOINTS; i++) {
        ASSERT_NEAR(expectedJointPositions[i], result[i], 1e-5);
    }
}

TEST_F(KinovaJaco2Should, returnJointVelocitiesIfApiCallSuccessful) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_TRUE(sut_->initialize());
    JointVelocities expectedJointVelocities(
        {-0.0872665, 0.0872665, 0.0872665, 0.0872665, 0.0872665, -0.0872665});
    ASSERT_NEAR(expectedJointVelocities[0], sut_->getJointVelocities().value()[0], 1e-4);
}

TEST_F(KinovaJaco2Should, returnJointForceTorquesIfApiCallSuccessful) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_TRUE(sut_->initialize());
    JointForceTorques expectedJointForceTorques({-5, 5, 5, 5, 5, -5});
    ASSERT_TRUE(areAlmostEqual(expectedJointForceTorques, sut_->getJointForceTorques().value()));
}

TEST_F(KinovaJaco2Should, returnArmPositionIfApiCallSuccessful) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::TaskPose taskPosBad(Eigen::Vector3d::Zero(),
    crf::math::rotation::CardanXYZ({0, 0, 0}));
    ASSERT_FALSE(areAlmostEqual(taskPosBad, sut_->getTaskPose().value()));
    crf::utility::types::TaskPose taskPosGood(Eigen::Vector3d({-0.3, -0.3, 0.2}),
        crf::math::rotation::CardanXYZ({0.1, 0.2, 0.2}));
    ASSERT_TRUE(areAlmostEqual(taskPosGood, sut_->getTaskPose().value(), 1e-7));
}

TEST_F(KinovaJaco2Should, returnFalseWhenSettersInvokeAndNotInitialized) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    JointVelocities jointVelocities({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_FALSE(sut_->setJointVelocities(false, jointVelocities));
    JointPositions jointPositions({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_FALSE(sut_->setJointPositions(false, jointPositions));
}

TEST_F(KinovaJaco2Should, returnFalseIfWrongInputJointsValuesSetJointVelocities) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_TRUE(sut_->initialize());
    JointVelocities jointVelocities({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_TRUE(sut_->setJointVelocities(false, jointVelocities));
    jointVelocities[0] = 100;
    ASSERT_FALSE(sut_->setJointVelocities(false, jointVelocities));
}

TEST_F(KinovaJaco2Should, setPositonMethodNotImplemented) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_TRUE(sut_->initialize());
    JointPositions jointPositions({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_FALSE(sut_->setJointPositions(false, jointPositions));
    ASSERT_FALSE(sut_->setJointPositions(false, jointPositions));
}

TEST_F(KinovaJaco2Should, returnFalseOrNonForNotAvailableInterfaceMethods) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(
        new KinovaJaco2(apiInterfaceMock_, KinovaJaco2Configuration(robotJSON))));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_FALSE(sut_->setJointPositions(false, sut_->getJointPositions().value()));
    ASSERT_FALSE(sut_->setJointForceTorques(false, JointForceTorques(KINOVA_JACO_NUM_JOINTS)));
    ASSERT_FALSE(sut_->setTaskPose(false, sut_->getTaskPose().value()));
    ASSERT_FALSE(sut_->setTaskVelocity(false, crf::utility::types::TaskVelocity()));
}
