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
#include "RobotBase/RobotBaseDefaultKinematics.hpp"

using testing::_;
using testing::Return;

using crf::actuators::robotbase::RobotBaseConfiguration;
using crf::utility::types::TaskVelocity;
using crf::actuators::robotbase::RobotBaseDefaultKinematics;

class RobotBaseDefaultKinematicsShould: public ::testing::Test {
 protected:
    RobotBaseDefaultKinematicsShould():
        logger_("RobotBaseDefaultKinematicsShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("RobotBase/"));
        testDirName_ += "RobotBase/tests/config/";
    }

    ~RobotBaseDefaultKinematicsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    RobotBaseConfiguration configuration_;
    std::unique_ptr<RobotBaseDefaultKinematics> kinematics_;
    std::string testDirName_;
};

TEST_F(RobotBaseDefaultKinematicsShould, failsIfRobotHasMoreThan4Wheels) {
    std::ifstream config(testDirName_ + "goodBotConfigWithLimits5Wheels.json");
    ASSERT_TRUE(configuration_.parse(nlohmann::json::parse(config)));
    ASSERT_THROW(kinematics_.reset(new RobotBaseDefaultKinematics(configuration_)),
        std::runtime_error);
}

TEST_F(RobotBaseDefaultKinematicsShould, correctlyBuildsWithGoodConfiguration) {
    std::ifstream config(testDirName_ + "goodCernBot2Config.json");
    ASSERT_TRUE(configuration_.parse(nlohmann::json::parse(config)));
    ASSERT_NO_THROW(kinematics_.reset(new RobotBaseDefaultKinematics(configuration_)));
}

TEST_F(RobotBaseDefaultKinematicsShould, failsIfInputVectorSizeIsIncorrect) {
    std::ifstream config(testDirName_ + "goodCernBot2Config.json");
    ASSERT_TRUE(configuration_.parse(nlohmann::json::parse(config)));
    ASSERT_NO_THROW(kinematics_.reset(new RobotBaseDefaultKinematics(configuration_)));

    std::vector<float> velocities({0, 2, 3});
    ASSERT_FALSE(kinematics_->getTaskVelocity(velocities));
}



TEST_F(RobotBaseDefaultKinematicsShould, correctlyWorksForwardAndInverseKinematics) {
    std::ifstream config(testDirName_ + "goodCernBot2Config.json");
    ASSERT_TRUE(configuration_.parse(nlohmann::json::parse(config)));
    ASSERT_NO_THROW(kinematics_.reset(new RobotBaseDefaultKinematics(configuration_)));

    TaskVelocity vel({1, -0.3, 0, 0, 0, 0.54});

    auto velocitiesWheels = kinematics_->getWheelsVelocity(vel);
    ASSERT_TRUE(velocitiesWheels);
    ASSERT_EQ(velocitiesWheels.get().size(), 4);
    auto returnVel = kinematics_->getTaskVelocity(velocitiesWheels.get());
    ASSERT_NEAR(returnVel.get()[0], 1, 1e-3);
    ASSERT_NEAR(returnVel.get()[1], -0.3, 1e-3);
    ASSERT_NEAR(returnVel.get()[5], 0.54, 1e-3);
}

TEST_F(RobotBaseDefaultKinematicsShould, correctlyWorksForwardKinematics) {
    std::ifstream config(testDirName_ + "goodCernBot2Config.json");
    ASSERT_TRUE(configuration_.parse(nlohmann::json::parse(config)));
    ASSERT_NO_THROW(kinematics_.reset(new RobotBaseDefaultKinematics(configuration_)));

    std::vector<float> velocities({1, 1, 1, 1});
    auto returnVel = kinematics_->getTaskVelocity(velocities);
    ASSERT_TRUE(returnVel);
    ASSERT_NEAR(returnVel.get()[0], 0.1016, 1e-3);
    ASSERT_NEAR(returnVel.get()[1], 0, 1e-3);
    ASSERT_NEAR(returnVel.get()[5], 0, 1e-3);

    velocities = std::vector<float>({1, -1, -0.5, 1});
    returnVel = kinematics_->getTaskVelocity(velocities);
    ASSERT_TRUE(returnVel);
    ASSERT_NEAR(returnVel.get()[0], 0.0127, 1e-3);
    ASSERT_NEAR(returnVel.get()[1], -0.0889, 1e-3);
    ASSERT_NEAR(returnVel.get()[5], -0.018, 1e-3);
}

TEST_F(RobotBaseDefaultKinematicsShould, correctlyWorksInverseKinematics) {
    std::ifstream config(testDirName_ + "goodCernBot2Config.json");
    ASSERT_TRUE(configuration_.parse(nlohmann::json::parse(config)));
    ASSERT_NO_THROW(kinematics_.reset(new RobotBaseDefaultKinematics(configuration_)));

    TaskVelocity velocity({1, -0.3658, 0, 0, 0, 2.54});
    auto returnVel = kinematics_->getWheelsVelocity(velocity);
    ASSERT_TRUE(returnVel);
    ASSERT_EQ(returnVel.get().size(), 4);

    ASSERT_NEAR(returnVel.get()[0], -3.994, 1e-3);
    ASSERT_NEAR(returnVel.get()[1], 23.6796, 1e-3);
    ASSERT_NEAR(returnVel.get()[2], -11.195, 1e-3);
    ASSERT_NEAR(returnVel.get()[3], 30.8804, 1e-3);
}

