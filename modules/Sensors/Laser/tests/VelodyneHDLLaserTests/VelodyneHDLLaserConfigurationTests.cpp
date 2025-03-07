/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <string>
#include <memory>
#include <vector>
#include <fstream>

#include "Laser/VelodyneHDL/VelodyneHDLLaserConfiguration.hpp"

class VelodyneHDLLaserConfigurationShould: public ::testing::Test {
 protected:
    VelodyneHDLLaserConfigurationShould(): logger_("VelodyneHDLLaserConfigurationShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("tests/"));
        testDirName_ += "tests/Configurations/Sensors/Laser/VelodyneHDLLaser/";
    }
    ~VelodyneHDLLaserConfigurationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    bool checkConfigEmptiness() {
        return (sut_->getLaserParameters().scanSize == 0);
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::sensors::laser::VelodyneHDLLaserConfiguration> sut_;
    std::string testDirName_;
};

TEST_F(VelodyneHDLLaserConfigurationShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::ifstream laserData(testDirName_ + "goodVelodyneLaserConfiguration.json");
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    ASSERT_NO_THROW(sut_.reset(new crf::sensors::laser::VelodyneHDLLaserConfiguration()));
    ASSERT_TRUE(sut_->parse(laserJSON));
    auto networkParameters = sut_->getNetworkConfiguration();
    ASSERT_EQ(networkParameters.ipAddress, "192.168.1.201");
    ASSERT_EQ(networkParameters.udpPort, 2368);
    ASSERT_EQ(sut_->getCorrectionParametersFilePath(), "");
}

TEST_F(VelodyneHDLLaserConfigurationShould, returnFalseAndBeEmptyAfterParsingBadConfigFile) {
    std::ifstream laserData(testDirName_ + "badVelodyneLaserConfigTypo.json");
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    ASSERT_NO_THROW(sut_.reset(new crf::sensors::laser::VelodyneHDLLaserConfiguration()));
    ASSERT_FALSE(sut_->parse(laserJSON));
    ASSERT_TRUE(checkConfigEmptiness());
}
