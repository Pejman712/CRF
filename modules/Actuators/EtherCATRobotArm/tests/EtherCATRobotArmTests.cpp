/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "EventLogger/EventLogger.hpp"
#include "EtherCATDevices/EtherCATMotorMock.hpp"
#include "EtherCATRobotArm/EtherCATRobotArm.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::ethercatdevices::EtherCATMotorMock;
using crf::actuators::ethercatrobotarm::EtherCATRobotArm;

class EtherCATRobotArmShould : public ::testing::Test {
 protected:
    EtherCATRobotArmShould() :
        logger_("EtherCATRobotArmShould"),
        motors_(),
        motorsEnabled_(),
        sut_(),
        configFileFolder_() {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        motors_.push_back(std::make_shared<NiceMock<EtherCATMotorMock> >());
        motors_.push_back(std::make_shared<NiceMock<EtherCATMotorMock> >());
        motors_.push_back(std::make_shared<NiceMock<EtherCATMotorMock> >());

        configFileFolder_ = __FILE__;
        configFileFolder_ = configFileFolder_.substr(0, configFileFolder_.find("tests"));
        configFileFolder_ += "tests/config/";

        motorsEnabled_.push_back(false);
        motorsEnabled_.push_back(false);
        motorsEnabled_.push_back(false);
    }

    void SetUp() override {
        for (size_t i = 0; i < motors_.size(); i++) {
            motorsEnabled_[i] = false;
            ON_CALL(*(motors_[i]), inFault()).WillByDefault(Return(false));
            ON_CALL(*(motors_[i]), faultReset()).WillByDefault(Return(true));
            ON_CALL(*(motors_[i]), shutdown()).WillByDefault(Return(true));
            ON_CALL(*(motors_[i]), enableOperation()).WillByDefault(Invoke([this, i]() {
                motorsEnabled_[i] = true;
                return true;
            }));
            ON_CALL(*(motors_[i]), isEnabled()).WillByDefault(Invoke([this, i]() -> std::optional<bool> {  // NOLINT
                bool value = motorsEnabled_[i];
                return value;
            }));
            ON_CALL(*(motors_[i]), disableOperation()).WillByDefault(Return(true));
            ON_CALL(*(motors_[i]), getPosition()).WillByDefault(Return(i*10));
            ON_CALL(*(motors_[i]), getVelocity()).WillByDefault(Return(i*20));
            ON_CALL(*(motors_[i]), getCurrent()).WillByDefault(Return(i*30));
            ON_CALL(*(motors_[i]), setPosition(_, _)).WillByDefault(Return(true));
            ON_CALL(*(motors_[i]), setVelocity(_)).WillByDefault(Return(true));
            ON_CALL(*(motors_[i]), setTorque(_)).WillByDefault(Return(true));
        }
    }

    ~EtherCATRobotArmShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::vector<std::shared_ptr<EtherCATMotorMock> > motors_;
    std::vector<bool> motorsEnabled_;
    const std::vector<double> jointConverFactors_ = {333771.788905168, 333771.788905168, 1000000};
    const double HDBelt_ = 1.6;

    std::unique_ptr<EtherCATRobotArm> sut_;
    std::string configFileFolder_;
};

TEST_F(EtherCATRobotArmShould, failsToInitializeOnBadConfigFile) {
    std::ifstream robotData(configFileFolder_ + "badEtherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATRobotArmShould, failsToInitializeIfCantCheckFault) {
    ON_CALL(*(motors_[0]), inFault()).WillByDefault(Return(std::nullopt));
    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATRobotArmShould, failsToInitializeIfCantResetFault) {
    ON_CALL(*(motors_[0]), inFault()).WillByDefault(Return(true));
    ON_CALL(*(motors_[0]), faultReset()).WillByDefault(Return(false));

    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATRobotArmShould, failsToInitializeIfCantShutdown) {
    ON_CALL(*(motors_[0]), shutdown()).WillByDefault(Return(false));

    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATRobotArmShould, failsToInitializeIfCantEnableOperation) {
    ON_CALL(*(motors_[0]), enableOperation()).WillByDefault(Return(false));

    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATRobotArmShould, failsToInitializeIfCantCheckEnable) {
    ON_CALL(*(motors_[0]), isEnabled()).WillByDefault(Return(std::nullopt));

    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATRobotArmShould, failsToInitializeIfDidntEnable) {
    ON_CALL(*(motors_[0]), isEnabled()).WillByDefault(Return(false));

    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATRobotArmShould, initializeDeinitializeSequence) {
    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(EtherCATRobotArmShould, operationFailsIfNotInitialized) {
    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_FALSE(sut_->getJointPositions());
    ASSERT_FALSE(sut_->getJointVelocities());
    ASSERT_FALSE(sut_->getJointForceTorques());
    ASSERT_FALSE(sut_->getTaskPose());
    ASSERT_FALSE(sut_->getTaskVelocity());

    ASSERT_FALSE(sut_->setJointPositions(crf::utility::types::JointPositions(3)));
    ASSERT_FALSE(sut_->setJointVelocities(crf::utility::types::JointVelocities(3)));
    ASSERT_FALSE(sut_->setJointForceTorques(crf::utility::types::JointForceTorques(3)));
    ASSERT_FALSE(sut_->setTaskPose(crf::utility::types::TaskPose()));
    ASSERT_FALSE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity(), false));
    ASSERT_FALSE(sut_->stopArm());
    ASSERT_FALSE(sut_->enableBrakes());
    ASSERT_FALSE(sut_->disableBrakes());
}

TEST_F(EtherCATRobotArmShould, correctlyGetAndSet) {
    for (size_t i = 0; i < motors_.size(); i++) {
        EXPECT_CALL(*(motors_[i]), setPosition(0, false)).Times(1);
        EXPECT_CALL(*(motors_[i]), setVelocity(0)).Times(1);
        EXPECT_CALL(*(motors_[i]), setTorque(0)).Times(1);
    }

    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_TRUE(sut_->initialize());
    auto position = sut_->getJointPositions();
    auto velocity = sut_->getJointVelocities();
    auto torque = sut_->getJointForceTorques();
    ASSERT_TRUE(position);
    ASSERT_TRUE(velocity);
    ASSERT_TRUE(torque);
    for (size_t i = 0; i < motors_.size(); i++) {
        if (i == 0) {
            ASSERT_NEAR(position.get()[i], i*10/(jointConverFactors_[i]*HDBelt_), 1e-5);
        } else {
            ASSERT_NEAR(position.get()[i], i*10/jointConverFactors_[i], 1e-5);
        }
        ASSERT_NEAR(velocity.get()[i], i*20/jointConverFactors_[i], 1e-5);
        ASSERT_NEAR(torque.get()[i], i*30, 1e-5);
    }

    ASSERT_TRUE(sut_->setJointPositions(crf::utility::types::JointPositions(3)));
    ASSERT_TRUE(sut_->setJointVelocities(crf::utility::types::JointVelocities(3)));
    ASSERT_TRUE(sut_->setJointForceTorques(crf::utility::types::JointForceTorques(3)));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(EtherCATRobotArmShould, failsGetAndSetIfHardwareFails) {
    EXPECT_CALL(*(motors_[0]), getPosition()).WillOnce(Return(10))
        .WillRepeatedly(Return(std::nullopt));
    ON_CALL(*(motors_[0]), getVelocity()).WillByDefault(Return(std::nullopt));
    ON_CALL(*(motors_[0]), getCurrent()).WillByDefault(Return(std::nullopt));
    ON_CALL(*(motors_[0]), setPosition(_, _)).WillByDefault(Return(false));
    ON_CALL(*(motors_[0]), setVelocity(_)).WillByDefault(Return(false));
    ON_CALL(*(motors_[0]), setTorque(_)).WillByDefault(Return(false));

    std::ifstream robotData(configFileFolder_ + "etherCATRobotArmConfig.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    sut_.reset(new EtherCATRobotArm(robotJSON, motors_[0], motors_[1], motors_[2]));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getJointPositions());
    ASSERT_FALSE(sut_->getJointVelocities());
    ASSERT_FALSE(sut_->getJointForceTorques());

    ASSERT_FALSE(sut_->setJointPositions(crf::utility::types::JointPositions(3)));
    ASSERT_FALSE(sut_->setJointVelocities(crf::utility::types::JointVelocities(3)));
    ASSERT_FALSE(sut_->setJointForceTorques(crf::utility::types::JointForceTorques(3)));

    ASSERT_FALSE(sut_->setJointPositions(crf::utility::types::JointPositions(2)));
    ASSERT_FALSE(sut_->setJointVelocities(crf::utility::types::JointVelocities(2)));
    ASSERT_FALSE(sut_->setJointForceTorques(crf::utility::types::JointForceTorques(2)));

    ASSERT_FALSE(sut_->setJointPositions(crf::utility::types::JointPositions(4)));
    ASSERT_FALSE(sut_->setJointVelocities(crf::utility::types::JointVelocities(4)));
    ASSERT_FALSE(sut_->setJointForceTorques(crf::utility::types::JointForceTorques(4)));
    ASSERT_TRUE(sut_->deinitialize());
}
