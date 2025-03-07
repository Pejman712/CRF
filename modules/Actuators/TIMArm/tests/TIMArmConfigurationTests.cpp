/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "TIMArm/TIMArmConfiguration.hpp"
#include "KinovaArm/KinovaArmConfiguration.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "Types/TaskTypes/TaskVelocity.hpp"
#include "Types/TaskTypes/TaskAcceleration.hpp"
#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;

class TIMArmConfigurationShould: public ::testing::Test {
 protected:
    TIMArmConfigurationShould() :
        logger_("TIMArmConfigurationShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        sut_ = std::make_unique<crf::actuators::timarm::TIMArmConfiguration>();
        configDirName_ = __FILE__;
        configDirName_ = configDirName_.substr(0, configDirName_.find("tests"));
        configDirName_ += "tests/config/";
    }

    ~TIMArmConfigurationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    bool checkConfigEmptiness() {
        return (
            sut_->getKinovaSerialNumber().empty() &&
            sut_->getKinovaNumberOfJoints() == 0 &&
            sut_->getKinovaNetworkConfiguration().localAddressIP.empty() &&
            sut_->getKinovaNetworkConfiguration().robotAddressIP.empty() &&
            sut_->getKinovaNetworkConfiguration().subnetMask.empty() &&
            sut_->getKinovaNetworkConfiguration().port == 0 &&
            sut_->getNumberOfJoints() == 0 &&
            sut_->getKinematicChain().empty() &&
            sut_->getJointsConfiguration().empty() &&
            sut_->getJointsDirection().empty() &&
            sut_->getJointsOffset().empty() &&
            sut_->getKinovaArmJSON().empty());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::actuators::timarm::TIMArmConfiguration> sut_;
    std::string configDirName_;
};

TEST_F(TIMArmConfigurationShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::ifstream robotData(configDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(sut_->parse(robotJSON));

    ASSERT_EQ(sut_->getHarmonicNumberOfJoints(), 3);

    ASSERT_EQ(sut_->getKinovaSerialNumber(), "PJ00900006518098-0 ");
    ASSERT_EQ(sut_->getKinovaNumberOfJoints(), 6);
    ASSERT_EQ(sut_->getKinovaNetworkConfiguration().localAddressIP, "192.168.3.41");
    ASSERT_EQ(sut_->getKinovaNetworkConfiguration().robotAddressIP,  "192.168.3.113");
    ASSERT_EQ(sut_->getKinovaNetworkConfiguration().subnetMask, "255.255.255.0");
    ASSERT_EQ(sut_->getKinovaNetworkConfiguration().port, 55000);

    ASSERT_EQ(sut_->getNumberOfJoints(), 9);
    ASSERT_EQ(sut_->getRTLoopTime().count(), 10);

    std::vector<crf::actuators::robotarm::DHParameter> kinematicChain = sut_->getKinematicChain();
    ASSERT_EQ(sut_->getNumberOfJoints(), kinematicChain.size());
    ASSERT_EQ(kinematicChain[0].type, crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(kinematicChain[1].type, crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(kinematicChain[2].type, crf::actuators::robotarm::DHParameter::JointType::Linear);
    ASSERT_EQ(kinematicChain[3].type, crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(kinematicChain[4].type, crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(kinematicChain[5].type, crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(kinematicChain[6].type, crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(kinematicChain[7].type, crf::actuators::robotarm::DHParameter::JointType::Rotational);
    ASSERT_EQ(kinematicChain[8].type, crf::actuators::robotarm::DHParameter::JointType::Rotational);

    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getJointsConfiguration().size());
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getJointsDirection().size());
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getJointsOffset().size());

    crf::utility::types::TaskVelocity vel({0.1, 0.1, 0.1, 1.0, 1.0, 1.0});
    crf::utility::types::TaskAcceleration acc({0.1, 0.1, 0.1, 1.0, 1.0, 1.0});
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumVelocity, vel, 10e-5));
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumAcceleration, acc, 10e-5));

    std::ifstream kinovaData(configDirName_ + "kinovaGoodConfiguration.json");
    nlohmann::json kinovaJSON = nlohmann::json::parse(kinovaData);
    ASSERT_EQ(sut_->getKinovaArmJSON(), kinovaJSON);

    std::ifstream harmonicData(configDirName_ + "etherCATGoodConfiguration.json");
    nlohmann::json harmonicJSON = nlohmann::json::parse(harmonicData);
    ASSERT_EQ(sut_->getHarmonicArmJSON(), harmonicJSON);
}

TEST_F(TIMArmConfigurationShould, beAbleToParseTheSameConfigFileTwice) {
    std::ifstream robotData(configDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(sut_->parse(robotJSON));
    ASSERT_TRUE(sut_->parse(robotJSON));
    ASSERT_EQ(sut_->getNumberOfJoints(), 9);
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getJointsConfiguration().size());
    ASSERT_EQ(sut_->getNumberOfJoints(), sut_->getKinematicChain().size());
    ASSERT_EQ(sut_->getRTLoopTime().count(), 10);

    crf::utility::types::TaskVelocity vel({0.1, 0.1, 0.1, 1.0, 1.0, 1.0});
    crf::utility::types::TaskAcceleration acc({0.1, 0.1, 0.1, 1.0, 1.0, 1.0});
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumVelocity, vel, 10e-5));
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskLimits().maximumAcceleration, acc, 10e-5));
}

TEST_F(TIMArmConfigurationShould, returnFalseAndBeEmptyAfterParsingBadConfigFile) {
    std::ifstream robotData1(
        configDirName_ + "badJointsOrientationConfiguration.json");
    nlohmann::json robotJSON1 = nlohmann::json::parse(robotData1);
    ASSERT_FALSE(sut_->parse(robotJSON1));
    ASSERT_TRUE(checkConfigEmptiness());


    std::ifstream robotData4(
        configDirName_ + "badNetworkConfiguration.json");
    nlohmann::json robotJSON4 = nlohmann::json::parse(robotData4);
    ASSERT_FALSE(sut_->parse(robotJSON4));
    ASSERT_TRUE(checkConfigEmptiness());

    std::ifstream robotData5(
        configDirName_ + "badSerialNumberConfiguration.json");
    nlohmann::json robotJSON5 = nlohmann::json::parse(robotData5);
    ASSERT_FALSE(sut_->parse(robotJSON5));
    ASSERT_TRUE(checkConfigEmptiness());
}

TEST_F(TIMArmConfigurationShould, returnFalseIfNumerOfJointsDoesNotMatch) {
    std::ifstream robotData(
        configDirName_ + "badJointsNumberConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_FALSE(sut_->parse(robotJSON));
    ASSERT_TRUE(checkConfigEmptiness());
}
