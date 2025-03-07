/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <bitset>
#include <map>
#include <memory>
#include <fstream>
#include <string>
#include <vector>
#include <thread>

#include <nlohmann/json.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "CANopenDrivers/CoESOEMAPIMockConfiguration.hpp"
#include "CANopenDrivers/CoEMaster/CoEMaster.hpp"
#include "CANopenDrivers/CiA402/CoEDrivers/CiA402CoEDriver/CiA402CoEDriver.hpp"

#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

using crf::devices::canopendrivers::CoESOEMAPIMockConfiguration;
using crf::devices::canopendrivers::CoEMaster;
using crf::devices::canopendrivers::CiA301;
using crf::devices::canopendrivers::CiA402;
using crf::devices::canopendrivers::ControlWord;
using crf::devices::canopendrivers::StatusWord;
using crf::devices::canopendrivers::ModeOfOperation;
using crf::devices::canopendrivers::StatusWordMask;
using crf::devices::canopendrivers::ControlWordMask;
using crf::devices::canopendrivers::CiA402CoEDriver;

class CiA402CoEDriverShould : public ::testing::Test {
 protected:
    CiA402CoEDriverShould() :
        logger_("CiA402CoEDriverShould"),
        numberOfSlaves_(1) {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~CiA402CoEDriverShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        logger_->info("SetUp");
        soem_ = std::make_shared<NiceMock<CoESOEMAPIMockConfiguration>>(
            numberOfSlaves_, IOMapSize_);
        soem_->configure();

        std::string testFileDirName = __FILE__;
        testFileDirName = testFileDirName.substr(0, testFileDirName.find("modules/"));
        testFileDirName += "modules/Devices/CANopenDrivers/tests/config/CoE/ELMOGoldSoloTwitter.json";  // NOLINT
        std::ifstream f(testFileDirName);
        data_ = nlohmann::json::parse(f);

        master_ = std::make_unique<CoEMaster>(
            "ifname", numberOfSlaves_, cycleTime_, data_, IOMapSize_, soem_);
        master_->initialize();

        sut_ = std::make_unique<CiA402CoEDriver>(master_, 1, data_);
    }

    void TearDown() override {
        logger_->info("TearDown");
    }

    const int numberOfSlaves_ = 2;
    const std::chrono::microseconds cycleTime_ = std::chrono::microseconds(1000);
    const int IOMapSize_ = 32;

    std::shared_ptr<CoESOEMAPIMockConfiguration> soem_;
    std::shared_ptr<CoEMaster> master_;
    std::unique_ptr<CiA402CoEDriver> sut_;
    nlohmann::json data_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(CiA402CoEDriverShould, initalizeTheSlaveAndMoveItToOperationEnabled) {
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402CoEDriverShould, goIntoFaultAtSwitchOnDisabled) {
    soem_->resetValueTransitions();
    soem_->goIntoFaultAutomaticTransition();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CoEDriverShould, goIntoFaultAtReadyToSwitchOn) {
    soem_->resetValueTransitions();
    soem_->goIntoFaultShutdown();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CoEDriverShould, goIntoFaultAtSwitchOn) {
    soem_->resetValueTransitions();
    soem_->goIntoFaultSwitchOn();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CoEDriverShould, goIntoFaultAtEnableOperation) {
    soem_->resetValueTransitions();
    soem_->goIntoFaultEnableOperation();
    ASSERT_FALSE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CoEDriverShould, goIntoFaultDeinitialize) {
    soem_->resetValueTransitions();
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);
    soem_->goIntoFaultDisableVoltage();
    ASSERT_FALSE(sut_->deinitialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}

TEST_F(CiA402CoEDriverShould, goIntoQuickStop) {
    soem_->resetValueTransitions();
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);

    ASSERT_TRUE(sut_->quickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::QuickStopActive);
}

TEST_F(CiA402CoEDriverShould, goIntoResetQuickStop) {
    soem_->resetValueTransitions();
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);


    ASSERT_TRUE(sut_->quickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::QuickStopActive);


    ASSERT_TRUE(sut_->resetQuickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);
}

TEST_F(CiA402CoEDriverShould, goIntoFaultAtQuickStop) {
    soem_->resetValueTransitions();
    soem_->goIntoFaultQuickStop();
    ASSERT_TRUE(sut_->initialize());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::OperationEnabled);
    ASSERT_FALSE(sut_->quickStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_EQ(sut_->getStatusWord(), StatusWord::Fault);
}
