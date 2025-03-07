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
#include <chrono>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "CANopenDrivers/CoESOEMAPIMockConfiguration.hpp"
#include "CANopenDrivers/CoEMaster/CoEMaster.hpp"
#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::canopendrivers::CoEMaster;
using crf::devices::canopendrivers::CoESOEMAPIMockConfiguration;

class CoEMasterShould : public ::testing::Test {
 protected:
    CoEMasterShould() :
        logger_("CoEMasterShould") {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~CoEMasterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        logger_->debug("SetUp");
        soemMock_ = std::make_shared<NiceMock<CoESOEMAPIMockConfiguration>>(
            numberOfSlaves_, IOMapSize_);
        soemMock_->configure();

        std::string testFileDirName = __FILE__;
        testFileDirName = testFileDirName.substr(0, testFileDirName.find("modules/"));
        testFileDirName += "modules/Devices/CANopenDrivers/tests/config/CoE/ELMOGoldSoloTwitter.json";  // NOLINT
        std::ifstream f(testFileDirName);
        data_ = nlohmann::json::parse(f);

        sut_ = std::make_unique<CoEMaster>(
            "ifname", numberOfSlaves_, cycleTime_, data_, IOMapSize_, soemMock_);
    }

    void TearDown() override {
        logger_->debug("TearDown");
    }

    const int numberOfSlaves_ = 2;
    const std::chrono::microseconds cycleTime_ = std::chrono::microseconds(1000);
    const int IOMapSize_ = 32;

    std::shared_ptr<CoESOEMAPIMockConfiguration> soemMock_;
    std::unique_ptr<CoEMaster> sut_;
    nlohmann::json data_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(CoEMasterShould, configurePDOsCorrectly) {
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(CoEMasterShould, configurePDOsCorrectlyWithSeveralSlaves) {
    std::string testFileDirName = __FILE__;
    testFileDirName = testFileDirName.substr(0, testFileDirName.find("modules/"));
    testFileDirName += "modules/Devices/CANopenDrivers/tests/config/CoE/ELMOGoldSoloTwitterSeveral.json";  // NOLINT
    std::ifstream f(testFileDirName);
    data_ = nlohmann::json::parse(f);

    sut_ = std::make_unique<CoEMaster>(
        "ifname", 2, cycleTime_, data_, IOMapSize_, soemMock_);
    ASSERT_TRUE(sut_->initialize());
}
