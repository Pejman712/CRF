/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <bitset>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "EtherCATDevices/EtherCATManager.hpp"
#include "EtherCATDevices/EtherCATMotor.hpp"
#include "EtherCATDevices/EtherCATDef.hpp"
#include "EtherCATDevices/SoemSimulator.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

using crf::devices::ethercatdevices::EtherCATManager;
using crf::devices::ethercatdevices::EtherCATMotor;
using crf::devices::ethercatdevices::SoemApiMock;

class EtherCATMotorShould : public ::testing::Test {
 protected:
    EtherCATMotorShould() :
        logger_("EtherCATMotorShould"),
        soemMock_(new NiceMock<SoemApiMock>),
        soem_(new SoemSimulator(soemMock_, 2)),
        numberOfSlaves_(2),
        manager_(new EtherCATManager("ifname", numberOfSlaves_, soem_->ioMapSize_, 0, soemMock_)),
        motor1_() {
            logger_->info("{} BEGIN",
                      testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~EtherCATMotorShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }


    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<SoemApiMock> soemMock_;
    std::unique_ptr<SoemSimulator> soem_;

    int numberOfSlaves_;
    std::shared_ptr<EtherCATManager> manager_;

    std::unique_ptr<EtherCATMotor> motor1_;
};

TEST_F(EtherCATMotorShould, correctlyReturnNodeId) {
    motor1_.reset(new EtherCATMotor(1, manager_));
    ASSERT_EQ(motor1_->getID(), 1);
}

TEST_F(EtherCATMotorShould, failsToInitializeIfManagerIsNotInitialized) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_FALSE(motor1_->initialize());
}


TEST_F(EtherCATMotorShould, initializeDeinitializeSequence) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_FALSE(motor1_->initialize());
    ASSERT_TRUE(motor1_->deinitialize());
    ASSERT_FALSE(motor1_->deinitialize());

    ASSERT_TRUE(motor1_->initialize());
    ASSERT_FALSE(motor1_->initialize());
    ASSERT_TRUE(motor1_->deinitialize());
    ASSERT_FALSE(motor1_->deinitialize());
}

TEST_F(EtherCATMotorShould, initFailsIfNotPreOp) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    soem_->ec_slave_[1].state = EC_STATE_PRE_OP - 1;
    ASSERT_FALSE(motor1_->initialize());
}

TEST_F(EtherCATMotorShould, initFailsIFWriteSdoFails) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    soem_->failToWriteSdo_ = true;
    ASSERT_FALSE(motor1_->initialize());
}

TEST_F(EtherCATMotorShould, cantBindPdosIfNotInitialized) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_FALSE(motor1_->bindPDOs());
}


TEST_F(EtherCATMotorShould, cantBindPdosIfIOMapNotConfigured) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_FALSE(motor1_->bindPDOs());
}


TEST_F(EtherCATMotorShould, correctlyBindPdosSequence) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->deinitialize());
}

TEST_F(EtherCATMotorShould, allOperationFailsIfNotInitialized) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_FALSE(motor1_->getEtherCatState());
    ASSERT_FALSE(motor1_->isAlive());
    ASSERT_FALSE(motor1_->inFault());
    ASSERT_FALSE(motor1_->inQuickStop());
    ASSERT_FALSE(motor1_->isEnabled());
    ASSERT_FALSE(motor1_->isReadyToSwitchOn());
    ASSERT_FALSE(motor1_->isSwitchOnDisabled());
    ASSERT_FALSE(motor1_->isSwitchedOn());
    ASSERT_FALSE(motor1_->enableOperation());
    ASSERT_FALSE(motor1_->disableOperation());
    ASSERT_FALSE(motor1_->disableVoltage());
    ASSERT_FALSE(motor1_->stop());
    ASSERT_FALSE(motor1_->quickStop());
    ASSERT_FALSE(motor1_->shutdown());
    ASSERT_FALSE(motor1_->faultReset());
    ASSERT_FALSE(motor1_->targetReached());
    ASSERT_FALSE(motor1_->getVelocity());
    ASSERT_FALSE(motor1_->getPosition());
    ASSERT_FALSE(motor1_->getCurrent());
    ASSERT_FALSE(motor1_->getStatusWord());
    ASSERT_FALSE(motor1_->getModeOfOperation());
    ASSERT_FALSE(motor1_->getDigitalInput(0));
    ASSERT_FALSE(motor1_->getTorque());
    ASSERT_FALSE(motor1_->getAnalogInput());
    ASSERT_FALSE(motor1_->setModeOfOperation(0));
    ASSERT_FALSE(motor1_->setDigitalOutput(0));
    ASSERT_FALSE(motor1_->resetDigitalOutput(0));
    ASSERT_FALSE(motor1_->setPosition(0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, 0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, 0, 0, false));
    ASSERT_FALSE(motor1_->setVelocity(0));
    ASSERT_FALSE(motor1_->setVelocity(0, 0));
    ASSERT_FALSE(motor1_->setVelocity(0, 0, 0));
    ASSERT_FALSE(motor1_->setTorque(0));
    ASSERT_FALSE(motor1_->setMaxCurrent(0));
    ASSERT_FALSE(motor1_->setProfileVelocity(0));
    ASSERT_FALSE(motor1_->setProfileAcceleration(0));
    ASSERT_FALSE(motor1_->setProfileDeceleration(0));
    ASSERT_FALSE(motor1_->setMaxVelocity(0));
    ASSERT_FALSE(motor1_->setMaxAcceleration(0));
    ASSERT_FALSE(motor1_->setMaxDeceleration(0));
    ASSERT_FALSE(motor1_->setQuickstopDeceleration(0));
    ASSERT_FALSE(motor1_->setPositionLimits({0, 0}));
    ASSERT_FALSE(motor1_->setMotorRatedCurrent(0));
    ASSERT_FALSE(motor1_->getProfileVelocity());
    ASSERT_FALSE(motor1_->getProfileAcceleration());
    ASSERT_FALSE(motor1_->getProfileDeceleration());
    ASSERT_FALSE(motor1_->getQuickstopDeceleration());
    ASSERT_FALSE(motor1_->getMaxVelocity());
    ASSERT_FALSE(motor1_->getMaxAcceleration());
    ASSERT_FALSE(motor1_->getMaxDeceleration());
    ASSERT_FALSE(motor1_->getPositionLimits());
}

TEST_F(EtherCATMotorShould, allOperationFailsIfNotLinkedPdo) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->isAlive());
    ASSERT_FALSE(motor1_->inFault());
    ASSERT_FALSE(motor1_->inQuickStop());
    ASSERT_FALSE(motor1_->isEnabled());
    ASSERT_FALSE(motor1_->isReadyToSwitchOn());
    ASSERT_FALSE(motor1_->isSwitchOnDisabled());
    ASSERT_FALSE(motor1_->isSwitchedOn());
    ASSERT_FALSE(motor1_->enableOperation());
    ASSERT_FALSE(motor1_->disableOperation());
    ASSERT_FALSE(motor1_->disableVoltage());
    ASSERT_FALSE(motor1_->stop());
    ASSERT_FALSE(motor1_->quickStop());
    ASSERT_FALSE(motor1_->shutdown());
    ASSERT_FALSE(motor1_->faultReset());
    ASSERT_FALSE(motor1_->targetReached());
    ASSERT_FALSE(motor1_->getVelocity());
    ASSERT_FALSE(motor1_->getPosition());
    ASSERT_FALSE(motor1_->getCurrent());
    ASSERT_FALSE(motor1_->getStatusWord());
    ASSERT_FALSE(motor1_->getModeOfOperation());
    ASSERT_FALSE(motor1_->getDigitalInput(0));
    ASSERT_FALSE(motor1_->getTorque());
    ASSERT_FALSE(motor1_->getAnalogInput());
    ASSERT_FALSE(motor1_->setModeOfOperation(0));
    ASSERT_FALSE(motor1_->setDigitalOutput(0));
    ASSERT_FALSE(motor1_->resetDigitalOutput(0));
    ASSERT_FALSE(motor1_->setPosition(0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, 0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, 0, 0, false));
    ASSERT_FALSE(motor1_->setVelocity(0));
    ASSERT_FALSE(motor1_->setVelocity(0, 0));
    ASSERT_FALSE(motor1_->setVelocity(0, 0, 0));
    ASSERT_FALSE(motor1_->setTorque(0));
    ASSERT_FALSE(motor1_->setMaxTorque(0));
}

TEST_F(EtherCATMotorShould, allOperationFailsIfNotInOperation) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_FALSE(motor1_->isAlive());
    ASSERT_FALSE(motor1_->inFault());
    ASSERT_FALSE(motor1_->inQuickStop());
    ASSERT_FALSE(motor1_->isEnabled());
    ASSERT_FALSE(motor1_->isReadyToSwitchOn());
    ASSERT_FALSE(motor1_->isSwitchOnDisabled());
    ASSERT_FALSE(motor1_->isSwitchedOn());
    ASSERT_FALSE(motor1_->enableOperation());
    ASSERT_FALSE(motor1_->disableOperation());
    ASSERT_FALSE(motor1_->disableVoltage());
    ASSERT_FALSE(motor1_->stop());
    ASSERT_FALSE(motor1_->quickStop());
    ASSERT_FALSE(motor1_->shutdown());
    ASSERT_FALSE(motor1_->faultReset());
    ASSERT_FALSE(motor1_->targetReached());
    ASSERT_FALSE(motor1_->getVelocity());
    ASSERT_FALSE(motor1_->getPosition());
    ASSERT_FALSE(motor1_->getCurrent());
    ASSERT_FALSE(motor1_->getStatusWord());
    ASSERT_FALSE(motor1_->getModeOfOperation());
    ASSERT_FALSE(motor1_->getDigitalInput(0));
    ASSERT_FALSE(motor1_->getTorque());
    ASSERT_FALSE(motor1_->getAnalogInput());
    ASSERT_FALSE(motor1_->setModeOfOperation(0));
    ASSERT_FALSE(motor1_->setDigitalOutput(0));
    ASSERT_FALSE(motor1_->resetDigitalOutput(0));
    ASSERT_FALSE(motor1_->setPosition(0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, 0, false));
    ASSERT_FALSE(motor1_->setPosition(0, 0, 0, 0, false));
    ASSERT_FALSE(motor1_->setVelocity(0));
    ASSERT_FALSE(motor1_->setVelocity(0, 0));
    ASSERT_FALSE(motor1_->setVelocity(0, 0, 0));
    ASSERT_FALSE(motor1_->setTorque(0));
    ASSERT_FALSE(motor1_->setMaxTorque(0));
}

TEST_F(EtherCATMotorShould, inFaultFlagTest) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::InFault_Value;
    ASSERT_TRUE(motor1_->inFault().value());
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::IsEnabled_Value;
    ASSERT_FALSE(motor1_->inFault().value());
}

TEST_F(EtherCATMotorShould, inQuickStopFlagTest) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::InQuickStop_Value;
    ASSERT_TRUE(motor1_->inQuickStop().value());
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::IsEnabled_Value;
    ASSERT_FALSE(motor1_->inQuickStop().value());
}

TEST_F(EtherCATMotorShould, enabledFlagTest) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::IsEnabled_Value;
    ASSERT_TRUE(motor1_->isEnabled().value());
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::InQuickStop_Value;
    ASSERT_FALSE(motor1_->isEnabled().value());
}

TEST_F(EtherCATMotorShould, readyToSwitchOnFlagTest) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].statusWord =
        crf::devices::ethercatdevices::statusword::IsReadyToSwitchOn_Value;
    ASSERT_TRUE(motor1_->isReadyToSwitchOn().value());
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::InQuickStop_Value;
    ASSERT_FALSE(motor1_->isReadyToSwitchOn().value());
}

TEST_F(EtherCATMotorShould, switchOnDisabledFlagTest) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].statusWord =
        crf::devices::ethercatdevices::statusword::IsSwitchOnDisabled_Value;
    ASSERT_TRUE(motor1_->isSwitchOnDisabled().value());
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::InQuickStop_Value;
    ASSERT_FALSE(motor1_->isSwitchOnDisabled().value());
}

TEST_F(EtherCATMotorShould, switchOnFlagTest) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].statusWord =
        crf::devices::ethercatdevices::statusword::IsSwitchedOn_Value;
    ASSERT_TRUE(motor1_->isSwitchedOn().value());
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::InQuickStop_Value;
    ASSERT_FALSE(motor1_->isSwitchedOn().value());
}

TEST_F(EtherCATMotorShould, correctlyGetVelocity) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].velocityActualValue = 123;
    ASSERT_EQ(motor1_->getVelocity().value(), 123);
}

TEST_F(EtherCATMotorShould, correctlyGetPosition) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].positionActualValue = 123;
    ASSERT_EQ(motor1_->getPosition().value(), 123);
}

TEST_F(EtherCATMotorShould, correctlyGetCurrent) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].currentActualValue = 123;
    ASSERT_EQ(motor1_->getCurrent().value(), 123);
}

TEST_F(EtherCATMotorShould, correctlyGetTorque) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].torqueActualValue = 123;
    ASSERT_EQ(motor1_->getTorque().value(), 123);
}

TEST_F(EtherCATMotorShould, correctlyGetStatusWord) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].statusWord = 123;
    ASSERT_EQ(motor1_->getStatusWord().value(), 123);
}

TEST_F(EtherCATMotorShould, correctlyGetModeOfOperation) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].modeOfOperationDisplay = 123;
    ASSERT_EQ(motor1_->getModeOfOperation().value(), 123);
}

TEST_F(EtherCATMotorShould, correctlyGetAnalogueInput) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].analogInput = 123;
    ASSERT_EQ(motor1_->getAnalogInput().value(), 123);
}

TEST_F(EtherCATMotorShould, correctlyGetDigitalInput) {
    ON_CALL(*soemMock_, send_processdata()).WillByDefault(Return(0));
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].digitalInputs = 0x00010000;
    ASSERT_TRUE(motor1_->getDigitalInput(1).value());
}

TEST_F(EtherCATMotorShould, correctlySetModeOfOperation) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    ASSERT_TRUE(motor1_->setModeOfOperation(
        crf::devices::ethercatdevices::modesofoperation::ProfilePositionMode));
}

TEST_F(EtherCATMotorShould, correctlySetAndGetSDOs) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());

    ASSERT_TRUE(motor1_->setProfileVelocity(100));
    ASSERT_EQ(motor1_->getProfileVelocity().value(), 100);

    ASSERT_TRUE(motor1_->setProfileAcceleration(100));
    ASSERT_EQ(motor1_->getProfileAcceleration().value(), 100);

    ASSERT_TRUE(motor1_->setProfileDeceleration(100));
    ASSERT_EQ(motor1_->getProfileDeceleration().value(), 100);

    ASSERT_TRUE(motor1_->setMaxVelocity(100));
    ASSERT_EQ(motor1_->getMaxVelocity().value(), 100);

    ASSERT_TRUE(motor1_->setMaxAcceleration(100));
    ASSERT_EQ(motor1_->getMaxAcceleration().value(), 100);

    ASSERT_TRUE(motor1_->setMaxDeceleration(100));
    ASSERT_EQ(motor1_->getMaxDeceleration().value(), 100);

    ASSERT_TRUE(motor1_->setQuickstopDeceleration(100));
    ASSERT_EQ(motor1_->getQuickstopDeceleration().value(), 100);

    ASSERT_TRUE(motor1_->setMotorRatedCurrent(100));
    ASSERT_EQ(motor1_->getMotorRatedCurrent().value(), 100);

    ASSERT_TRUE(motor1_->setPositionLimits({0, 100}));
    ASSERT_EQ(motor1_->getPositionLimits().value().first, 0);
    ASSERT_EQ(motor1_->getPositionLimits().value().second, 100);
}

TEST_F(EtherCATMotorShould, enableOperationTests) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->outputs_[0].controlWord = 0x0080;
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::IsSwitchedOn_Value;
    ASSERT_TRUE(motor1_->enableOperation());
}

TEST_F(EtherCATMotorShould, disableOperationTests) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->outputs_[0].controlWord = 0x0080;
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::IsEnabled_Value;
    ASSERT_TRUE(motor1_->disableOperation());
}

TEST_F(EtherCATMotorShould, disableVoltageTests) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->outputs_[0].controlWord = 0x0080;
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::IsEnabled_Value;
    ASSERT_TRUE(motor1_->disableVoltage());
}

TEST_F(EtherCATMotorShould, quickStopTests) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->outputs_[0].controlWord = 0x0080;
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::IsEnabled_Value;
    ASSERT_TRUE(motor1_->quickStop());
}

TEST_F(EtherCATMotorShould, shutdownTests) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->outputs_[0].controlWord = 0x0080;
    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::IsEnabled_Value;
    ASSERT_TRUE(motor1_->shutdown());
}

TEST_F(EtherCATMotorShould, faultResetTests) {
    motor1_.reset(new EtherCATMotor(1, manager_));

    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));

    soem_->inputs_[0].statusWord = crf::devices::ethercatdevices::statusword::InFault_Value;
    ASSERT_TRUE(motor1_->faultReset());
}

TEST_F(EtherCATMotorShould, correctlySetPosition1) {
    motor1_.reset(new EtherCATMotor(1, manager_));
    ASSERT_FALSE(motor1_->setPosition(10000, true));
    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_FALSE(motor1_->setPosition(10000, true));
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->setMaxCurrent(50000));
    ASSERT_TRUE(motor1_->setModeOfOperation(crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode));
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_TRUE(motor1_->enableOperation());
    ASSERT_TRUE(motor1_->setProfileAcceleration(200));
    ASSERT_TRUE(motor1_->setProfileDeceleration(200));
    ASSERT_TRUE(motor1_->setProfileVelocity(1000));
    ASSERT_TRUE(motor1_->setPosition(10000, true));
    ASSERT_NE(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_FALSE(motor1_->targetReached().value());
    ASSERT_TRUE(motor1_->stop());
    ASSERT_TRUE(motor1_->targetReached().value());
}

TEST_F(EtherCATMotorShould, correctlySetPosition2) {
    motor1_.reset(new EtherCATMotor(1, manager_));
    ASSERT_FALSE(motor1_->setPosition(10000, true));
    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_FALSE(motor1_->setPosition(10000, true));
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->setMaxCurrent(50000));
    ASSERT_TRUE(motor1_->setModeOfOperation(crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode));
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_TRUE(motor1_->enableOperation());
    ASSERT_TRUE(motor1_->setProfileAcceleration(200));
    ASSERT_TRUE(motor1_->setProfileDeceleration(200));
    ASSERT_TRUE(motor1_->setProfileVelocity(1000));
    ASSERT_TRUE(motor1_->setPosition(10000, 10, true));
    ASSERT_NE(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_EQ(motor1_->getProfileVelocity().value(), 10);
    ASSERT_FALSE(motor1_->targetReached().value());
    ASSERT_TRUE(motor1_->stop());
    ASSERT_TRUE(motor1_->targetReached().value());
}

TEST_F(EtherCATMotorShould, correctlySetPosition3) {
    motor1_.reset(new EtherCATMotor(1, manager_));
    ASSERT_FALSE(motor1_->setPosition(10000, true));
    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_FALSE(motor1_->setPosition(10000, true));
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->setMaxCurrent(50000));
    ASSERT_TRUE(motor1_->setModeOfOperation(crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode));
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_TRUE(motor1_->enableOperation());
    ASSERT_TRUE(motor1_->setProfileAcceleration(200));
    ASSERT_TRUE(motor1_->setProfileDeceleration(200));
    ASSERT_TRUE(motor1_->setProfileVelocity(1000));
    ASSERT_TRUE(motor1_->setPosition(10000, 10, 20, true));
    ASSERT_NE(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_EQ(motor1_->getProfileVelocity().value(), 10);
    ASSERT_EQ(motor1_->getProfileAcceleration().value(), 20);
    ASSERT_FALSE(motor1_->targetReached().value());
    ASSERT_TRUE(motor1_->stop());
    ASSERT_TRUE(motor1_->targetReached().value());
}

TEST_F(EtherCATMotorShould, correctlySetPosition4) {
    motor1_.reset(new EtherCATMotor(1, manager_));
    ASSERT_FALSE(motor1_->setPosition(10000, true));
    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_FALSE(motor1_->setPosition(10000, true));
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->setMaxCurrent(50000));
    ASSERT_TRUE(motor1_->setModeOfOperation(crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode));
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_TRUE(motor1_->enableOperation());
    ASSERT_TRUE(motor1_->setProfileAcceleration(200));
    ASSERT_TRUE(motor1_->setProfileDeceleration(200));
    ASSERT_TRUE(motor1_->setProfileVelocity(1000));
    ASSERT_TRUE(motor1_->setPosition(10000, 10, 20, 30, true));
    ASSERT_NE(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_EQ(motor1_->getProfileVelocity().value(), 10);
    ASSERT_EQ(motor1_->getProfileAcceleration().value(), 20);
    ASSERT_EQ(motor1_->getProfileDeceleration().value(), 30);
    ASSERT_FALSE(motor1_->targetReached().value());
    ASSERT_TRUE(motor1_->stop());
    ASSERT_TRUE(motor1_->targetReached().value());
}

TEST_F(EtherCATMotorShould, correctlySetVelocity1) {
    motor1_.reset(new EtherCATMotor(1, manager_));
    ASSERT_FALSE(motor1_->setVelocity(100));
    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_FALSE(motor1_->setVelocity(100));
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->setMaxCurrent(50000));
    ASSERT_TRUE(motor1_->setModeOfOperation(crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode));
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_TRUE(motor1_->enableOperation());
    ASSERT_TRUE(motor1_->setProfileAcceleration(200));
    ASSERT_TRUE(motor1_->setProfileDeceleration(200));
    ASSERT_TRUE(motor1_->setVelocity(100));
    ASSERT_NE(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_TRUE(motor1_->stop());
    ASSERT_TRUE(motor1_->targetReached().value());
}

TEST_F(EtherCATMotorShould, correctlySetVelocity2) {
    motor1_.reset(new EtherCATMotor(1, manager_));
    ASSERT_FALSE(motor1_->setVelocity(100));
    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_FALSE(motor1_->setVelocity(100));
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->setMaxCurrent(50000));
    ASSERT_TRUE(motor1_->setModeOfOperation(crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode));
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_TRUE(motor1_->enableOperation());
    ASSERT_TRUE(motor1_->setProfileAcceleration(200));
    ASSERT_TRUE(motor1_->setProfileDeceleration(200));
    ASSERT_TRUE(motor1_->setVelocity(100, 10));
    ASSERT_NE(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_EQ(motor1_->getProfileAcceleration().value(), 10);
    ASSERT_TRUE(motor1_->stop());
    ASSERT_TRUE(motor1_->targetReached().value());
}

TEST_F(EtherCATMotorShould, correctlySetVelocity3) {
    motor1_.reset(new EtherCATMotor(1, manager_));
    ASSERT_FALSE(motor1_->setVelocity(100));
    ASSERT_TRUE(manager_->initialize());
    ASSERT_TRUE(motor1_->initialize());
    ASSERT_TRUE(manager_->configureIOMap());
    ASSERT_FALSE(motor1_->setVelocity(100));
    ASSERT_TRUE(motor1_->bindPDOs());
    ASSERT_TRUE(manager_->enterOp(0));
    ASSERT_TRUE(motor1_->setMaxCurrent(50000));
    ASSERT_TRUE(motor1_->setModeOfOperation(crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode));
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_TRUE(motor1_->enableOperation());
    ASSERT_TRUE(motor1_->setProfileAcceleration(200));
    ASSERT_TRUE(motor1_->setProfileDeceleration(200));
    ASSERT_TRUE(motor1_->setVelocity(100, 10, 20));
    ASSERT_NE(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfilePositionMode);
    ASSERT_EQ(motor1_->getModeOfOperation(), crf::devices::ethercatdevices::
    modesofoperation::ProfileVelocityMode);
    ASSERT_EQ(motor1_->getProfileAcceleration().value(), 10);
    ASSERT_EQ(motor1_->getProfileDeceleration().value(), 20);
    ASSERT_TRUE(motor1_->stop());
    ASSERT_TRUE(motor1_->targetReached().value());
}
