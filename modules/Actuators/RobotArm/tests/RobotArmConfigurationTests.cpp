/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "Types/TaskTypes/TaskVelocity.hpp"
#include "Types/TaskTypes/TaskAcceleration.hpp"

using testing::_;
using testing::Return;

class RobotArmConfigurationShould: public ::testing::Test {
 protected:
    RobotArmConfigurationShould(): logger_("RobotArmConfigurationShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        sut_ = std::make_unique<crf::actuators::robotarm::RobotArmConfiguration>();
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("RobotArm"));
        testDirName_ += "RobotArm/tests/config/";
    }
    ~RobotArmConfigurationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    bool checkConfigEmptiness() {
        return (sut_->getNumberOfJoints() == 0 &&
            sut_->getJointsConfiguration().empty() &&
            sut_->getKinematicChain().empty());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::actuators::robotarm::RobotArmConfiguration> sut_;
    std::string testDirName_;
};

TEST_F(RobotArmConfigurationShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::ifstream robotData(testDirName_ + "goodConfigurationLinear.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(sut_->parse(robotJSON));
    ASSERT_EQ(sut_->getNumberOfJoints(), 6);
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getJointsConfiguration().size());
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getKinematicChain().size());
    ASSERT_EQ(sut_->getRTLoopTime().count(), 10);

    crf::utility::types::TaskVelocity vel({0.1, 0.1, 0.1, 1.0, 1.0, 1.0});
    crf::utility::types::TaskAcceleration acc({0.1, 0.1, 0.1, 1.0, 1.0, 1.0});

    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumVelocity, vel, 10e-5));
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumAcceleration, acc, 10e-5));

    ASSERT_EQ(sut_->getKinematicChain()[0].type,
        crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(sut_->getKinematicChain()[1].type,
        crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(sut_->getKinematicChain()[2].type,
        crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(sut_->getKinematicChain()[3].type,
        crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(sut_->getKinematicChain()[4].type,
        crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(sut_->getKinematicChain()[5].type,
        crf::actuators::robotarm::DHParameter::JointType::Linear);
}

TEST_F(RobotArmConfigurationShould, beAbleToParseTheSameConfigFileTwice) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(sut_->parse(robotJSON));
    ASSERT_TRUE(sut_->parse(robotJSON));
    ASSERT_EQ(sut_->getNumberOfJoints(), 6);
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getJointsConfiguration().size());
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getKinematicChain().size());
    ASSERT_EQ(sut_->getRTLoopTime().count(), 10);

    crf::utility::types::TaskVelocity vel({0.1, 0.1, 0.1, 1.0, 1.0, 1.0});
    crf::utility::types::TaskAcceleration acc({0.1, 0.1, 0.1, 1.0, 1.0, 1.0});

    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumVelocity, vel, 10e-5));
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumAcceleration, acc, 10e-5));
}

TEST_F(RobotArmConfigurationShould, returnFalseAndBeEmptyAfterParsingBadConfigFile) {
    std::ifstream robotData1(testDirName_ + "badConfigurationDHArray.json");
    nlohmann::json robotJSON1 = nlohmann::json::parse(robotData1);
    ASSERT_FALSE(sut_->parse(robotJSON1));
    ASSERT_TRUE(checkConfigEmptiness());

    std::ifstream robotData2(testDirName_ + "badConfigurationJointsArray.json");
    nlohmann::json robotJSON2 = nlohmann::json::parse(robotData2);
    ASSERT_FALSE(sut_->parse(robotJSON2));
    ASSERT_TRUE(checkConfigEmptiness());

    std::ifstream robotData3(testDirName_ + "badTaskVelocityLimits.json");
    nlohmann::json robotJSON3 = nlohmann::json::parse(robotData3);
    ASSERT_FALSE(sut_->parse(robotJSON3));
    ASSERT_TRUE(checkConfigEmptiness());

    std::ifstream robotData4(testDirName_ + "badTaskAccelerationLimits.json");
    nlohmann::json robotJSON4 = nlohmann::json::parse(robotData4);
    ASSERT_FALSE(sut_->parse(robotJSON4));
    ASSERT_TRUE(checkConfigEmptiness());
}

TEST_F(RobotArmConfigurationShould, readRobotLengthsForwardKinematicsAndJacobian) {
    // Correct results
    std::ifstream robotData(testDirName_ + "goodConfigurationFKLengthsJacobian.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(sut_->parse(robotJSON));

    ASSERT_EQ(sut_->getNumberOfJoints(), 6);
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getJointsConfiguration().size());
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getKinematicChain().size());
    ASSERT_EQ(sut_->getRTLoopTime().count(), 2);

    crf::utility::types::TaskVelocity vel({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::TaskAcceleration acc({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumVelocity, vel, 10e-5));
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumAcceleration, acc, 10e-5));

    std::array<std::vector<double>, 3> robotLengths = sut_->getRobotLengths();
    std::vector<double> lx = {0.0, 0.0, 0.6127, 0.57155, 0.0, 0.0};
    std::vector<double> ly = {0.0, 0.0, 0.0, 0.0, 0.0, 0.11655};
    std::vector<double> lz = {0.0, 0.1807, 0.0, 0.0, 0.17415, 0.11985};
    ASSERT_EQ(robotLengths[0], lx);
    ASSERT_EQ(robotLengths[1], ly);
    ASSERT_EQ(robotLengths[2], lz);

    ASSERT_EQ(sut_->getFKMathExpressions(), robotJSON["KinematicChain"]["parameters2"]);
    ASSERT_EQ(sut_->getJacobianMathExpressions(), nlohmann::json(robotJSON["Jacobian"]));


    // Errors
    std::ifstream wrongRobotData1(testDirName_ + "badLengthsConfiguration1.json");
    nlohmann::json wrongRobotJSON1 = nlohmann::json::parse(wrongRobotData1);
    ASSERT_FALSE(sut_->parse(wrongRobotJSON1));

    std::ifstream wrongRobotData2(testDirName_ + "badLengthsConfiguration2.json");
    nlohmann::json wrongRobotJSON2 = nlohmann::json::parse(wrongRobotData2);
    ASSERT_FALSE(sut_->parse(wrongRobotJSON2));

    std::ifstream wrongRobotData3(testDirName_ + "withoutLengthsConfiguration.json");
    nlohmann::json wrongRobotJSON3 = nlohmann::json::parse(wrongRobotData3);
    ASSERT_FALSE(sut_->parse(wrongRobotJSON3));

    std::ifstream wrongRobotData4(testDirName_ + "withoutFKConfiguration.json");
    nlohmann::json wrongRobotJSON4 = nlohmann::json::parse(wrongRobotData4);
    ASSERT_FALSE(sut_->parse(wrongRobotJSON4));

    std::ifstream wrongRobotData5(testDirName_ + "withoutJacobianConfiguration.json");
    nlohmann::json wrongRobotJSON5 = nlohmann::json::parse(wrongRobotData5);
    ASSERT_TRUE(sut_->parse(wrongRobotJSON5));
    ASSERT_NE(sut_->getJacobianMathExpressions(), nlohmann::json(robotJSON["Jacobian"]));
}
