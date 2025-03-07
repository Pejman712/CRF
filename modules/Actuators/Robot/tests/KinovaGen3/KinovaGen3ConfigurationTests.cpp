/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Shuqi Zhao CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <regex>
#include <string>
#include <fstream>
#include <vector>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "Robot/KinovaGen3/KinovaGen3Configuration.hpp"

using testing::_;
using testing::Return;

using crf::actuators::robot::KinovaGen3Configuration;

class KinovaGen3ConfigurationShould: public ::testing::Test {
 protected:
    KinovaGen3ConfigurationShould() {
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
            testDirName_.find("KinovaGen3ConfigurationTests.cpp"));
        testDirName_ += "config/";
    }
    bool checkConfigEmptiness() {
        return (
            kinovagen3configuration_->getIPAddress().empty() &&
            kinovagen3configuration_->getTCPPort() == 0 &&
            kinovagen3configuration_->getUDPPort() == 0 &&
            kinovagen3configuration_->getSessionTimeout() == 0 &&
            kinovagen3configuration_->getConnectionTimeout() == 0);
    }
    std::unique_ptr<KinovaGen3Configuration> kinovagen3configuration_;
    std::string testDirName_;
};

TEST_F(KinovaGen3ConfigurationShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::ifstream robotData(testDirName_ + "KinovaGen3.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(kinovagen3configuration_.reset(new KinovaGen3Configuration(robotJSON)));
    std::vector<double> maxpos = {6.28, 2.27, 2.57, 6.28, 2.09, 6.28};
    std::vector<double> minpos = {-6.28, -2.27, -2.57, -6.28, -2.09, -6.28};
    std::vector<double> maxvel = {0.8727, 0.8727, 0.8727, 0.8727, 0.8727, 0.8727};
    ASSERT_EQ(kinovagen3configuration_->getIPAddress(), "192.168.1.10");
    ASSERT_EQ(kinovagen3configuration_->getJointSpaceDoF(), 6);
    ASSERT_EQ(kinovagen3configuration_->getTCPPort(), 10000);
    ASSERT_EQ(kinovagen3configuration_->getUDPPort(), 10001);
    ASSERT_EQ(kinovagen3configuration_->getJointLimits().maxPosition[3], maxpos[3]);
    ASSERT_EQ(kinovagen3configuration_->getJointLimits().minPosition[2], minpos[2]);
    ASSERT_EQ(kinovagen3configuration_->getJointLimits().maxVelocity[1], maxvel[1]);
}

TEST_F(KinovaGen3ConfigurationShould, returnFalseAndBeEmptyAfterParsingBadConfigFile) {
    std::ifstream robotData1(testDirName_ + "missingtimeout.json");
    nlohmann::json robotJSON1 = nlohmann::json::parse(robotData1);
    ASSERT_THROW(kinovagen3configuration_.reset(
        new KinovaGen3Configuration(robotJSON1)), std::invalid_argument);

    std::ifstream robotData2(testDirName_ + "missingIP.json");
    nlohmann::json robotJSON2 = nlohmann::json::parse(robotData2);
    ASSERT_THROW(kinovagen3configuration_.reset(
        new KinovaGen3Configuration(robotJSON2)), std::invalid_argument);
}

TEST_F(KinovaGen3ConfigurationShould, ThrowExceptionIfSizeIsWrongInTorqueOrOrientenation) {
    std::ifstream robotData3(testDirName_ + "wrongMaxpos.json");
    nlohmann::json robotJSON3 = nlohmann::json::parse(robotData3);
    ASSERT_THROW(kinovagen3configuration_.reset(
        new KinovaGen3Configuration(robotJSON3)), std::invalid_argument);
}
