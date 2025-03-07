/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <thread>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"
#include "SOEMAPI/SOEMAPIMockConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::ethercatdrivers::EtherCATMaster;
using crf::communication::soemapi::SOEMAPIMockConfiguration;

class EtherCATMasterShould : public ::testing::Test {
 protected:
    EtherCATMasterShould() :
        soemMock_(std::make_shared<NiceMock<SOEMAPIMockConfiguration>>(
            numberOfSlaves_, ioMapSize_)),
        logger_("EtherCATMasterShould") {
            logger_->info("{} BEGIN",
                      testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~EtherCATMasterShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        soemMock_->configure();
        sut_ = std::make_unique<EtherCATMaster>(
            "ifname", 5, std::chrono::microseconds(1000), 4096, soemMock_);
    }

    void TearDown() override {
        logger_->info("TearDown");
    }

    const int numberOfSlaves_ = 5;
    const int ioMapSize_ = 32;

    std::shared_ptr<SOEMAPIMockConfiguration> soemMock_;
    std::unique_ptr<EtherCATMaster> sut_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(EtherCATMasterShould, throwsExceptionIfSlavesLessThan1) {
    ASSERT_THROW(
        std::make_unique<EtherCATMaster>(
            "ifname", 0, std::chrono::microseconds(1000), ioMapSize_, soemMock_),
        std::invalid_argument);
}

TEST_F(EtherCATMasterShould, throwsExceptionIfCycleTimeLessThan0) {
    ASSERT_THROW(
        std::make_unique<EtherCATMaster>(
            "ifname", numberOfSlaves_, std::chrono::microseconds(0), ioMapSize_, soemMock_),
        std::invalid_argument);
}

TEST_F(EtherCATMasterShould, throwsExceptionIfIOSizeLessThan0) {
    ASSERT_THROW(
        std::make_unique<EtherCATMaster>(
            "ifname", numberOfSlaves_, std::chrono::microseconds(1000), -4, soemMock_),
        std::invalid_argument);
}

TEST_F(EtherCATMasterShould, initializeSuccessfully) {
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(EtherCATMasterShould, failInitializeIfItCantConfigInit) {
    EXPECT_CALL(*soemMock_, config_init(_)).WillOnce(Return(0));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATMasterShould, initializeFailsIfDifferentSlavesWhereFound) {
    EXPECT_CALL(*soemMock_, ec_slavecount()).WillRepeatedly(Return(numberOfSlaves_-1));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATMasterShould, failToConfigureIOMapBiggerSize) {
    EXPECT_CALL(*soemMock_, config_overlap_map(_)).WillOnce(Return(6969));  // Nice
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATMasterShould, failToMoveSlavesToOperational) {
    soemMock_->forceSlaveState(0, EC_STATE_PRE_OP);
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(EtherCATMasterShould, failToRetrieveOutputsWhenInitializeFails) {
    soemMock_->forceSlaveState(0, EC_STATE_PRE_OP);
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->retrieveOutputs(0));
}

TEST_F(EtherCATMasterShould, failToRetrieveOutputsWhenTheIDOfTheSlaveIsOutOfTheScope) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->retrieveOutputs(numberOfSlaves_+1));
}

TEST_F(EtherCATMasterShould, successfullyRetrieveOutputs) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(std::optional<uint8*>(soemMock_->ec_slave()[0].outputs), sut_->retrieveOutputs(0));
}

TEST_F(EtherCATMasterShould, failToRetrieveInputsWhenConfigIOMapFails) {
    soemMock_->forceSlaveState(0, EC_STATE_PRE_OP);
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->retrieveInputs(0));
}

TEST_F(EtherCATMasterShould, failToRetriveInputsWhenTheIDOfTheSlaveIsOutOfTheScope) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->retrieveInputs(numberOfSlaves_+1));
}

TEST_F(EtherCATMasterShould, successfullyRetrieveInputs) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(std::optional<uint8*>(soemMock_->ec_slave()[0].inputs), sut_->retrieveInputs(0));
}

TEST_F(EtherCATMasterShould, failToPutASlaveToGoIntoOperationalMode) {
    ASSERT_TRUE(sut_->initialize());
    soemMock_->forceSlaveState(1, EC_STATE_ERROR);
    ASSERT_FALSE(sut_->goIntoOperationalMode(1));
}

TEST_F(EtherCATMasterShould, successfullyPutASlaveToGoIntoOperationalMode) {
    ASSERT_TRUE(sut_->initialize());
    int ind = 1+(rand() % numberOfSlaves_); //  NOLINT
    ASSERT_TRUE(sut_->goIntoOperationalMode(ind));
}

TEST_F(EtherCATMasterShould, returnTheCorrectState) {
    ASSERT_TRUE(sut_->initialize());
    int ind = 1+(rand() % numberOfSlaves_); //  NOLINT
    ASSERT_EQ(soemMock_->ec_slave()[ind].state, sut_->readSlaveState(ind));
}

TEST_F(EtherCATMasterShould, failToCommunicateWithSlavesWhenSlaveIDIsOutOfScope) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->slaveCommunicationCheck(numberOfSlaves_ + 1));
}

TEST_F(EtherCATMasterShould, failToCommunicateWithSlavesWhenNotInitialized) {
    ASSERT_FALSE(sut_->slaveCommunicationCheck(0));
}

TEST_F(EtherCATMasterShould, successfullyCommunicateWithSlaves) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->goIntoOperationalMode(0));
    ASSERT_TRUE(sut_->slaveCommunicationCheck(0));
}
