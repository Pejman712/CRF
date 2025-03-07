/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>

#include "EventLogger/EventLogger.hpp"
#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "EtherCATDevices/SoemApiMock.hpp"
#include "EtherCATDevices/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.hpp"
#include "EtherCATDevices/SoemSimulator.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::EtherCATMotor;
using crf::devices::ethercatdevices::SoemApiMock;
using crf::devices::ethercatdevices::TIMRobotArmWagonMotors;

class TIMRobotArmWagonMotorsShould : public ::testing::Test {
 protected:
    TIMRobotArmWagonMotorsShould() :
        logger_("TIMRobotArmWagonMotorsShould"),
        soemMock_(new NiceMock<SoemApiMock>),
        soem_(new SoemSimulator(soemMock_, 5)) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~TIMRobotArmWagonMotorsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<SoemApiMock> soemMock_;
    std::unique_ptr<SoemSimulator> soem_;
    std::unique_ptr<TIMRobotArmWagonMotors> sut_;
};

TEST_F(TIMRobotArmWagonMotorsShould, initializeDeinitializeSequence) {
    ASSERT_NO_THROW(sut_.reset(new TIMRobotArmWagonMotors("eno1", 4096, soemMock_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    EXPECT_CALL(*soemMock_, statecheck(_, _, _)).WillOnce(Invoke(
        [this](uint16_t slave, uint16_t reqstate, int timeout) {
            for (int i=0; i < soem_->numberOfSlaves_ + 1; i++) {
                soem_->ec_slave_[i].state = EC_STATE_PRE_OP;
            }
            return 2;
        })).WillRepeatedly(DoDefault());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMRobotArmWagonMotorsShould, initializeFailsIfCantInitSoem) {
    ON_CALL(*soemMock_, config_init(false)).WillByDefault(Return(0));
    ASSERT_NO_THROW(sut_.reset(new TIMRobotArmWagonMotors("eno1", 4096, soemMock_)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMRobotArmWagonMotorsShould, initializeFailsIfCantConfigInit) {
    ON_CALL(*soemMock_, config_init(false)).WillByDefault(Return(0));
    ASSERT_NO_THROW(sut_.reset(new TIMRobotArmWagonMotors("eno1", 4096, soemMock_)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMRobotArmWagonMotorsShould, initializeFailsIfOneMotorNotFound) {
    ON_CALL(*soemMock_, ec_slavecount()).WillByDefault(Return(2));
    ASSERT_NO_THROW(sut_.reset(new TIMRobotArmWagonMotors("eno1", 4096, soemMock_)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMRobotArmWagonMotorsShould, initializeFailsIfOneMotorFailsSDO) {
    ON_CALL(*soemMock_, SDOwrite(_, _, _, _, _, _, _)).WillByDefault(Return(0));
    ASSERT_NO_THROW(sut_.reset(new TIMRobotArmWagonMotors("eno1", 4096, soemMock_)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMRobotArmWagonMotorsShould, initFailsIfMotorNotPreOp) {
    ASSERT_NO_THROW(sut_.reset(new TIMRobotArmWagonMotors("eno1", 4096, soemMock_)));

    soem_->ec_slave_[1].state = 0;
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMRobotArmWagonMotorsShould, retrieveManagerBeforeInit) {
    ASSERT_NO_THROW(sut_.reset(new TIMRobotArmWagonMotors("eno1", 4096, soemMock_)));

    ASSERT_FALSE(sut_->getManager());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(TIMRobotArmWagonMotorsShould, retrieveMotorsBeforeInit) {
    ASSERT_NO_THROW(sut_.reset(new TIMRobotArmWagonMotors("eno1", 4096, soemMock_)));

    ASSERT_FALSE(sut_->getHarmonicDrive1());
    ASSERT_FALSE(sut_->getHarmonicDrive2());
    ASSERT_FALSE(sut_->getLinearSled());
    ASSERT_FALSE(sut_->getStabilizer());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->getHarmonicDrive1());
    ASSERT_TRUE(sut_->getHarmonicDrive2());
    ASSERT_TRUE(sut_->getLinearSled());
    ASSERT_TRUE(sut_->getStabilizer());
    ASSERT_TRUE(sut_->getShielding());
}
