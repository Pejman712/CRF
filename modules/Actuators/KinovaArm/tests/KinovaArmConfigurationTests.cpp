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

#include "KinovaArm/KinovaArmConfiguration.hpp"

using testing::_;
using testing::Return;

using crf::actuators::kinovaarm::KinovaArmConfiguration;

class KinovaArmConfigurationShould: public ::testing::Test {
 protected:
    KinovaArmConfigurationShould() {
        configuration_ = std::make_unique<KinovaArmConfiguration>();
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
            testDirName_.find("KinovaArmConfigurationTests.cpp"));
        testDirName_ += "config/";
    }
    bool checkConfigEmptiness() {
        return (
            configuration_->getNumberOfJoints() == 0 &&
            configuration_->getJointsConfiguration().empty() &&
            configuration_->getKinematicChain().empty() &&
            configuration_->getSerialNumber().empty() &&
            configuration_->getNetworkConfiguration().localAddressIP.empty() &&
            configuration_->getNetworkConfiguration().robotAddressIP.empty() &&
            configuration_->getNetworkConfiguration().subnetMask.empty() &&
            configuration_->getNetworkConfiguration().port == 0 &&
            configuration_->getJointsDirection().empty());
    }
    std::unique_ptr<KinovaArmConfiguration> configuration_;
    std::string testDirName_;
};

TEST_F(KinovaArmConfigurationShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(configuration_->parse(robotJSON));
    ASSERT_EQ(configuration_->getNumberOfJoints(), 6);
    ASSERT_EQ(configuration_->getNumberOfJoints(), configuration_->getJointsConfiguration().size());
    ASSERT_EQ(configuration_->getNumberOfJoints(), configuration_->getKinematicChain().size());
    ASSERT_EQ(configuration_->getSerialNumber(), "PJ00900006509031-0 ");
    ASSERT_EQ(configuration_->getNetworkConfiguration().localAddressIP, "192.168.1.36");
    ASSERT_EQ(configuration_->getNetworkConfiguration().robotAddressIP,  "192.168.1.112");
    ASSERT_EQ(configuration_->getNetworkConfiguration().subnetMask, "255.255.255.0");
    ASSERT_EQ(configuration_->getNetworkConfiguration().port, 55000);
    ASSERT_EQ(configuration_->getNumberOfJoints(), configuration_->getJointsDirection().size());
}

TEST_F(KinovaArmConfigurationShould, beAbleToParseTheSameConfigFileTwice) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(configuration_->parse(robotJSON));
    ASSERT_TRUE(configuration_->parse(robotJSON));
    ASSERT_EQ(configuration_->getNumberOfJoints(), 6);
    ASSERT_EQ(configuration_->getNumberOfJoints(), configuration_->getJointsConfiguration().size());
    ASSERT_EQ(configuration_->getNumberOfJoints(), configuration_->getKinematicChain().size());
    ASSERT_EQ(configuration_->getSerialNumber(), "PJ00900006509031-0 ");
    ASSERT_EQ(configuration_->getNetworkConfiguration().localAddressIP, "192.168.1.36");
    ASSERT_EQ(configuration_->getNetworkConfiguration().robotAddressIP,  "192.168.1.112");
    ASSERT_EQ(configuration_->getNetworkConfiguration().subnetMask, "255.255.255.0");
    ASSERT_EQ(configuration_->getNetworkConfiguration().port, 55000);
    ASSERT_EQ(configuration_->getNumberOfJoints(), configuration_->getJointsDirection().size());
}

TEST_F(KinovaArmConfigurationShould, returnFalseAndBeEmptyAfterParsingBadConfigFile) {
    std::ifstream robotData1(testDirName_ + "badSerialNumberConfiguration.json");
    nlohmann::json robotJSON1 = nlohmann::json::parse(robotData1);
    ASSERT_FALSE(configuration_->parse(robotJSON1));
    ASSERT_TRUE(checkConfigEmptiness());

    std::ifstream robotData2(testDirName_ + "badNetworkConfiguration.json");
    nlohmann::json robotJSON2 = nlohmann::json::parse(robotData2);
    ASSERT_FALSE(configuration_->parse(robotJSON2));
    ASSERT_TRUE(checkConfigEmptiness());

    std::ifstream robotData5(testDirName_ + "badJointsOrientationConfiguration.json");
    nlohmann::json robotJSON5 = nlohmann::json::parse(robotData5);
    ASSERT_FALSE(configuration_->parse(robotJSON5));
    ASSERT_TRUE(checkConfigEmptiness());
}

TEST_F(KinovaArmConfigurationShould, returnTrueIfOptionalParametersAreNotGiven) {
    std::ifstream robotData3(testDirName_ + "badMaximumTorqueConfiguration.json");
    nlohmann::json robotJSON3 = nlohmann::json::parse(robotData3);
    ASSERT_TRUE(configuration_->parse(robotJSON3));
    ASSERT_FALSE(checkConfigEmptiness());

    std::ifstream robotData4(testDirName_ + "badMissingJointsOrientationConfiguration.json");
    nlohmann::json robotJSON4 = nlohmann::json::parse(robotData4);
    ASSERT_TRUE(configuration_->parse(robotJSON4));
    ASSERT_FALSE(checkConfigEmptiness());
}
