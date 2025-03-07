/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "RobotBase/RobotBaseConfiguration.hpp"

using testing::_;
using testing::Return;

using crf::actuators::robotbase::RobotBaseConfiguration;

class RobotBaseConfigurationShould: public ::testing::Test {
 protected:
    RobotBaseConfigurationShould():
        logger_("RobotBaseConfigurationShould") {
        configuration_ = std::make_unique<RobotBaseConfiguration>();
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("RobotBase/"));
        testDirName_ += "RobotBase/tests/config/";
    }
    bool checkConfigEmptiness() {
        return (
            configuration_->getNumberOfWheels() == 0);
    }
    ~RobotBaseConfigurationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<RobotBaseConfiguration> configuration_;
    std::string testDirName_;
};

TEST_F(RobotBaseConfigurationShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::ifstream config(testDirName_ + "goodCernBot2Config.json");
    ASSERT_TRUE(configuration_->parse(nlohmann::json::parse(config)));
    ASSERT_EQ(configuration_->getRTLoopTime(), 20000);

    ASSERT_EQ(configuration_->getNumberOfWheels(), 4);
    ASSERT_TRUE(configuration_->getRobotParameters().hasLiftingStage);
}

TEST_F(RobotBaseConfigurationShould, beAbleToParseTheSameConfigFileTwice) {
    std::ifstream config(testDirName_ + "goodCernBot2Config.json");
    ASSERT_TRUE(configuration_->parse(nlohmann::json::parse(config)));
    config.seekg(0);
    ASSERT_TRUE(configuration_->parse(nlohmann::json::parse(config)));
    ASSERT_EQ(configuration_->getRTLoopTime(), 20000);

    ASSERT_FLOAT_EQ(configuration_->getNumberOfWheels(), 4);
    ASSERT_FLOAT_EQ(configuration_->getRobotParameters().wheelsDiameter, 0.2032);
    ASSERT_FLOAT_EQ(configuration_->getRobotParameters().wheelsDistanceX, 0.685);
    ASSERT_FLOAT_EQ(configuration_->getTaskLimits().maximumVelocity[0], 2.0);
    ASSERT_FLOAT_EQ(configuration_->getTaskLimits().maximumVelocity[1], 2.0);
    ASSERT_FLOAT_EQ(configuration_->getTaskLimits().maximumVelocity[5], 1.0);
    ASSERT_FLOAT_EQ(configuration_->getRobotParameters().maximumWheelsVelocity, 15.7);
    ASSERT_FLOAT_EQ(configuration_->getRobotParameters().maximumWheelsAcceleration, 80);
    ASSERT_TRUE(configuration_->getRobotParameters().hasLiftingStage);
}

TEST_F(RobotBaseConfigurationShould, returnFalseAndBeEmptyAfterParsingBadConfigFile) {
    std::vector<std::string> filenames = {
        testDirName_ + "badConfig.json",
        testDirName_ + "badConfigWheelSize.json",
        testDirName_ + "badBotConfigWithWrongLimits.json"
    };
    for (const auto& filename : filenames) {
        std::ifstream config(filename);
        ASSERT_FALSE(configuration_->parse(nlohmann::json::parse(config)));
        ASSERT_TRUE(checkConfigEmptiness());
    }
}
