/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "KinovaArm/KinovaAdmittanceController.hpp"
#include "KinovaArm/KinovaJacoMock.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "RobotArmKinematics/IRobotArmKinematics.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"
#include "Types/Types.hpp"

#include "EventLogger/EventLogger.hpp"
#include <memory>
#include <string>
#include <fstream>

using crf::utility::logger::EventLogger;

using crf::actuators::kinovaarm::KinovaAdmittanceController;
using crf::actuators::kinovaarm::KinovaJacoMock;
using crf::actuators::robotarm::RobotArmConfiguration;
using crf::control::robotarmkinematics::IRobotArmKinematics;
using crf::control::robotarmkinematics::RobotArmKDLKinematics;
using crf::utility::types::JointPositions;
using crf::utility::types::JointForceTorques;
using crf::utility::types::areAlmostEqual;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class AdmittanceControllerShould: public ::testing::Test {
 protected:
    AdmittanceControllerShould() :
        logger_("AdmittanceControllerShould") {
        logger_->info("{} BEGIN",
                      testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
                                       testDirName_.find("KinovaAdmittanceControllerTests.cpp"));
        testDirName_ += "config";
    }
    void SetUp() override {
        robotArmConfiguration_.reset(new RobotArmConfiguration);
        std::ifstream robotData(testDirName_ + "/goodConfiguration.json");
        nlohmann::json robotJSON = nlohmann::json::parse(robotData);
        ASSERT_TRUE(robotArmConfiguration_->parse(robotJSON));

        kinematics_.reset(new RobotArmKDLKinematics(robotArmConfiguration_));
        arm_.reset(new NiceMock<KinovaJacoMock>(testDirName_ + "/goodConfiguration.json"));
        ON_CALL(*arm_, getConfiguration()).WillByDefault(Return(robotArmConfiguration_));
        ON_CALL(*arm_, getJointPositions()).WillByDefault(
                Invoke([this](){return generateDefaultJP();}));
        ON_CALL(*arm_, getJointForceTorques()).WillByDefault(
                Invoke([this](){return generateDefaultJT();}));
        ON_CALL(*arm_, moveHomePosition()).WillByDefault(Return(1));
        ON_CALL(*arm_, setJointPositions(_)).WillByDefault(Return(1));
    }

    boost::optional<JointPositions> generateDefaultJP() {
        JointPositions jointPositions(6);
        return jointPositions;
    }

    boost::optional<JointForceTorques> generateDefaultJT() {
        JointForceTorques jointForceTorques(6);
        jointForceTorques[2] = 0.6;
        jointForceTorques[4] = -6.2;
        return jointForceTorques;
    }

    ~AdmittanceControllerShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    EventLogger logger_;
    std::unique_ptr<KinovaAdmittanceController> sut_;
    std::string testDirName_;
    std::shared_ptr<IRobotArmKinematics> kinematics_;
    std::shared_ptr<RobotArmConfiguration> robotArmConfiguration_;
    std::shared_ptr<NiceMock<KinovaJacoMock>> arm_;
};

TEST_F(AdmittanceControllerShould, returnFalseIfWrongConfigFile) {
    ASSERT_NO_THROW(sut_.reset(new KinovaAdmittanceController(
        arm_, kinematics_, testDirName_+"badJointsOrientationConfiguration.json")));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(AdmittanceControllerShould, returnFalseIfDoubleInitDeinit) {
    ASSERT_NO_THROW(sut_.reset(new KinovaAdmittanceController(
            arm_, kinematics_, testDirName_+"/admittanceConfig.json")));
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(AdmittanceControllerShould, returnFalseIfSEtJointPositionsNotValid) {
    ASSERT_NO_THROW(sut_.reset(new KinovaAdmittanceController(
            arm_, kinematics_, testDirName_+"/admittanceConfig.json")));
    ASSERT_TRUE(sut_->initialize());
    JointPositions jointPositions(6);
    ASSERT_TRUE(sut_->setJointPositions(jointPositions));
    jointPositions[3] = -100;
    ASSERT_FALSE(sut_->setJointPositions(jointPositions));
    jointPositions[3] = 100;
    ASSERT_FALSE(sut_->setJointPositions(jointPositions));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(AdmittanceControllerShould, returnSameValueFromGetter) {
    ASSERT_NO_THROW(sut_.reset(new KinovaAdmittanceController(
            arm_, kinematics_, testDirName_+"/admittanceConfig.json")));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions(), generateDefaultJP().get()));
    ASSERT_TRUE(sut_->deinitialize());
}
