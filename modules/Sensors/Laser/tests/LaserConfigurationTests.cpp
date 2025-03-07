/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Laser/LaserConfiguration.hpp"

using testing::_;
using testing::Return;

class LaserConfigurationShould: public ::testing::Test {
 protected:
    LaserConfigurationShould():
    logger_("LaserConfigurationShould") {
        sut_ = std::make_unique<crf::sensors::laser::LaserConfiguration>();
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("tests/"));
        testDirName_ += "tests/Configurations/Sensors/Laser/HokuyoLaser/";
    }
    bool checkConfigEmptiness() {
        return (sut_->getLaserParameters().scanSize == 0);
    }
    ~LaserConfigurationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::sensors::laser::LaserConfiguration> sut_;
    std::string testDirName_;
};

TEST_F(LaserConfigurationShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::ifstream laserData(testDirName_ + "goodLaserConfig.json");
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    ASSERT_TRUE(sut_->parse(laserJSON));
    auto parameters = sut_->getLaserParameters();
    ASSERT_FLOAT_EQ(parameters.minRange, 0.020);
    ASSERT_FLOAT_EQ(parameters.maxRange, 5.600);
    ASSERT_FLOAT_EQ(parameters.minAngle, -119.53);
    ASSERT_FLOAT_EQ(parameters.maxAngle, 119.88);
    ASSERT_EQ(parameters.scanSize, 726);

    ASSERT_FLOAT_EQ(sut_->getAngularResolution(), 0.352);

    auto interval = std::chrono::microseconds(20000);
    sut_->setScanIntervalUS(interval);
    ASSERT_EQ(sut_->getScanIntervalUS().count(), 20000);
    crf::sensors::laser::LaserParameters params;
    params.scanSize = 1000;
    sut_->setLaserParameters(params);
    ASSERT_EQ(sut_->getLaserParameters().scanSize, params.scanSize);
}

TEST_F(LaserConfigurationShould, returnFalseAndBeEmptyAfterParsingBadConfigFile) {
    std::ifstream laserData1(testDirName_ + "badLaserConfigEmptyRange.json");
    nlohmann::json laserJSON1 = nlohmann::json::parse(laserData1);
    ASSERT_NO_THROW(sut_.reset(new crf::sensors::laser::LaserConfiguration()));
    ASSERT_FALSE(sut_->parse(laserJSON1));
    ASSERT_TRUE(checkConfigEmptiness());

    std::ifstream laserData2(testDirName_ + "badLaserConfigTypo.json");
    nlohmann::json laserJSON2 = nlohmann::json::parse(laserData2);
    ASSERT_NO_THROW(sut_.reset(new crf::sensors::laser::LaserConfiguration()));
    ASSERT_FALSE(sut_->parse(laserJSON2));
    ASSERT_TRUE(checkConfigEmptiness());

    std::ifstream laserData3(testDirName_ + "badLaserConfigWrongUSInterval.json");
    nlohmann::json laserJSON3 = nlohmann::json::parse(laserData3);
    ASSERT_NO_THROW(sut_.reset(new crf::sensors::laser::LaserConfiguration()));
    ASSERT_FALSE(sut_->parse(laserJSON3));
    ASSERT_TRUE(checkConfigEmptiness());
}
