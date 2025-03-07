/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO
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
#include <csignal>

#include "Laser/VelodyneHDL/VelodyneHDLLaser.hpp"

class VelodyneHDLLaserShould: public ::testing::Test {
 protected:
    VelodyneHDLLaserShould(): logger_("VelodyneHDLLaserShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("tests/"));
        testDirName_ += "tests/Configurations/Sensors/Laser/VelodyneHDLLaser/";
    }
    ~VelodyneHDLLaserShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::sensors::laser::VelodyneHDLLaser> sut_;
    std::string testDirName_;
};

TEST_F(VelodyneHDLLaserShould, returnEmptyAndFalseIfNotInitialized) {
    std::ifstream laserData(testDirName_ + "goodVelodyneLaserConfiguration.json");
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    ASSERT_NO_THROW(sut_.reset(new crf::sensors::laser::VelodyneHDLLaser(laserJSON)));
    ASSERT_EQ(nullptr, sut_->getPointCloud());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(VelodyneHDLLaserShould, DISABLED_returnFalseIfInitializedOrDeinitializedWithNoDevice) {
    std::ifstream laserData(testDirName_ + "goodVelodyneLaserConfiguration.json");
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    ASSERT_NO_THROW(sut_.reset(new crf::sensors::laser::VelodyneHDLLaser(laserJSON)));

    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(VelodyneHDLLaserShould, DISABLED_returnCorrectScanAfterInitialized) {
    std::ifstream laserData(testDirName_ + "goodVelodyneLaserConfiguration.json");
    nlohmann::json laserJSON = nlohmann::json::parse(laserData);
    ASSERT_NO_THROW(sut_.reset(new crf::sensors::laser::VelodyneHDLLaser(laserJSON)));
    ASSERT_TRUE(sut_->initialize());
    auto pointCloud = sut_->getPointCloud();
    ASSERT_TRUE(pointCloud);
    ASSERT_GT(pointCloud->size(), 0);
}
