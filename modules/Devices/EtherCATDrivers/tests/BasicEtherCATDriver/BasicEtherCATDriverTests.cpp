/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <map>
#include <memory>
#include <string>
#include <fstream>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"
#include "EtherCATDrivers/BasicEtherCATDriver/BasicEtherCATDriver.hpp"
#include "SOEMAPI/SOEMAPIMockConfiguration.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

using crf::communication::soemapi::ISOEMAPI;
using crf::communication::soemapi::SOEMAPIMockConfiguration;
using crf::devices::ethercatdrivers::EtherCATMaster;
using crf::devices::ethercatdrivers::BasicEtherCATDriver;

class BasicEtherCATDriverShould : public ::testing::Test {
 protected:
    BasicEtherCATDriverShould() :
        soemMock_(std::make_shared<NiceMock<SOEMAPIMockConfiguration>>(
            numberOfSlaves_, sizeOfIOMap_)),
        logger_("BasicEtherCATDriverShould") {
        logger_->info(
            "{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~BasicEtherCATDriverShould() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        soemMock_->configure();
        master_ = std::make_shared<EtherCATMaster>(
            "ifname", numberOfSlaves_, std::chrono::microseconds(1000), 4096, soemMock_);
        sut_ = std::make_unique<BasicEtherCATDriver>(master_, 1);
    }
    void TearDown() override {
        logger_->info("TearDown");
    }

    const int numberOfSlaves_ = 1;
    const int sizeOfIOMap_ = 32;

    std::shared_ptr<SOEMAPIMockConfiguration> soemMock_;
    std::shared_ptr<EtherCATMaster> master_;
    std::unique_ptr<BasicEtherCATDriver> sut_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(BasicEtherCATDriverShould, failToInitializeIfmMasterIsNotInitializeBefore) {
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(BasicEtherCATDriverShould, failToInitializeIfMasterIniFails) {
    soemMock_->forceSlaveState(0, EC_STATE_ERROR);
    ASSERT_FALSE(master_->initialize());
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(BasicEtherCATDriverShould, successfullyInitialize) {
    ASSERT_TRUE(master_->initialize());
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(BasicEtherCATDriverShould, successfullyReturnEtherCATState) {
    ASSERT_TRUE(master_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(master_->readSlaveState(1), sut_->getEtherCatState());
}

TEST_F(BasicEtherCATDriverShould, slaveShouldBeAlivePlease) {
    ASSERT_TRUE(master_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->isAlive());
}
