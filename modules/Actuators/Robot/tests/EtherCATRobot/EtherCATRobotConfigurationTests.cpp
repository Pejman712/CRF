/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: David Forkel CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Robot/EtherCATRobot/EtherCATRobotConfiguration.hpp"

using testing::_;
using testing::Return;

using crf::actuators::robot::EtherCATRobotConfiguration;

class EtherCATRobotConfigurationShould: public ::testing::Test {
 protected:
    EtherCATRobotConfigurationShould() {
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
            testDirName_.find("EtherCATRobotConfigurationTests.cpp"));
        testDirName_ += "config/";
    }

    std::unique_ptr<EtherCATRobotConfiguration> sut_;
    std::string testDirName_;
};

TEST_F(EtherCATRobotConfigurationShould, ParseValuesCorrectly) {
    std::ifstream robotData(testDirName_ + "MIRA1.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new EtherCATRobotConfiguration(robotJSON)));
    ASSERT_EQ(sut_->getJointSpaceDoF(), 4);
    ASSERT_EQ(sut_->getGearBoxReduction(), 1);
    TaskSpace taskSpace({true, true, false, false, false, true});
    ASSERT_TRUE(areEqual(sut_->getTaskSpace(), taskSpace));
}

TEST_F(EtherCATRobotConfigurationShould, FailIfNoRadToCount) {
    std::ifstream robotData(testDirName_ + "MIRA1_NoRadToCount.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_THROW(sut_.reset(new EtherCATRobotConfiguration(robotJSON)), std::invalid_argument);
}

TEST_F(EtherCATRobotConfigurationShould, FailIfNoMaxTorque) {
    std::ifstream robotData(testDirName_ + "MIRA1_NoMaxTorque.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_THROW(sut_.reset(new EtherCATRobotConfiguration(robotJSON)), std::invalid_argument);
}

TEST_F(EtherCATRobotConfigurationShould, FailIfNoMaxCurrent) {
    std::ifstream robotData(testDirName_ + "MIRA1_NoMaxCurrent.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_THROW(sut_.reset(new EtherCATRobotConfiguration(robotJSON)), std::invalid_argument);
}

TEST_F(EtherCATRobotConfigurationShould, SuccedIfNoGearBox) {
    std::ifstream robotData(testDirName_ + "MIRA1_NoGearBox.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new EtherCATRobotConfiguration(robotJSON)));
}
