/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Robot/KinovaJaco2/KinovaJaco2Configuration.hpp"

using testing::_;
using testing::Return;

using crf::actuators::robot::KinovaJaco2Configuration;

class KinovaJaco2ConfigurationShould: public ::testing::Test {
 protected:
    KinovaJaco2ConfigurationShould() {
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
            testDirName_.find("KinovaJaco2ConfigurationTests.cpp"));
        testDirName_ += "config/";
    }
    bool checkConfigEmptiness() {
        return (
            configuration_->getJointSpaceDoF() == 0 &&
            configuration_->getSerialNumber().empty() &&
            configuration_->getNetworkConfiguration().localAddressIP.empty() &&
            configuration_->getNetworkConfiguration().robotAddressIP.empty() &&
            configuration_->getNetworkConfiguration().subnetMask.empty() &&
            configuration_->getNetworkConfiguration().port == 0);
    }
    std::unique_ptr<KinovaJaco2Configuration> configuration_;
    std::string testDirName_;
};

TEST_F(KinovaJaco2ConfigurationShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    configuration_ = std::make_unique<KinovaJaco2Configuration>(robotJSON);
    ASSERT_EQ(configuration_->getJointSpaceDoF(), 6);
    ASSERT_EQ(configuration_->getSerialNumber(), "");
    ASSERT_EQ(configuration_->getNetworkConfiguration().localAddressIP, "192.168.0.36");
    ASSERT_EQ(configuration_->getNetworkConfiguration().robotAddressIP,  "192.168.0.111");
    ASSERT_EQ(configuration_->getNetworkConfiguration().subnetMask, "255.255.255.0");
    ASSERT_EQ(configuration_->getNetworkConfiguration().port, 55000);
}

TEST_F(KinovaJaco2ConfigurationShould, beAbleToParseTheSameConfigFileTwice) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    configuration_ = std::make_unique<KinovaJaco2Configuration>(robotJSON);
    ASSERT_EQ(configuration_->getJointSpaceDoF(), 6);
    ASSERT_EQ(configuration_->getSerialNumber(), "");
    ASSERT_EQ(configuration_->getNetworkConfiguration().localAddressIP, "192.168.0.36");
    ASSERT_EQ(configuration_->getNetworkConfiguration().robotAddressIP,  "192.168.0.111");
    ASSERT_EQ(configuration_->getNetworkConfiguration().subnetMask, "255.255.255.0");
    ASSERT_EQ(configuration_->getNetworkConfiguration().port, 55000);
}

TEST_F(KinovaJaco2ConfigurationShould, returnFalseAndBeEmptyAfterParsingBadConfigFile) {
    std::ifstream robotData1(testDirName_ + "badSerialNumberConfiguration.json");
    nlohmann::json robotJSON1 = nlohmann::json::parse(robotData1);
    ASSERT_THROW(configuration_.reset(
        new KinovaJaco2Configuration(robotJSON1)), std::invalid_argument);

    std::ifstream robotData2(testDirName_ + "badNetworkConfiguration.json");
    nlohmann::json robotJSON2 = nlohmann::json::parse(robotData2);
    ASSERT_THROW(configuration_.reset(
        new KinovaJaco2Configuration(robotJSON2)), std::invalid_argument);
}

TEST_F(KinovaJaco2ConfigurationShould, ThrowExceptionIfSizeIsWrongInTorqueOrOrientenation) {
    std::ifstream robotData3(testDirName_ + "badMaximumTorqueConfiguration.json");
    nlohmann::json robotJSON3 = nlohmann::json::parse(robotData3);
    ASSERT_THROW(configuration_.reset(
        new KinovaJaco2Configuration(robotJSON3)), std::invalid_argument);
}
