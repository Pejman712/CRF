/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "FTSensorCalibrator/FTSensorCalibratorConfig.hpp"

using testing::_;
using testing::Return;

using crf::applications::ftsensorcalibrator::FtSensorCalibratorConfig;

class FtSensorCalibratorConfigShould: public ::testing::Test {
 protected:
    FtSensorCalibratorConfigShould() {
        configuration_ = std::make_unique<FtSensorCalibratorConfig>();
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0,
            testDirName_.find("FTSensorCalibrationConfigurationTests.cpp"));
        testDirName_ += "config/";
    }
    std::unique_ptr<FtSensorCalibratorConfig> configuration_;
    std::string testDirName_;
};

TEST_F(FtSensorCalibratorConfigShould, containCorrectValuesAfterParsinGoodConfigFile) {
    std::cout << " " << testDirName_ + "SchunkArmCalibration.json" << " " << std::endl;
    ASSERT_TRUE(configuration_->parse(testDirName_ + "SchunkArmCalibration.json"));
    ASSERT_EQ(configuration_->getNumberOfJoints(), 6);
    ASSERT_EQ(configuration_->getNumberOfJoints(), configuration_->getInitialJP().size());
    ASSERT_EQ(configuration_->getNumberOfJoints(), configuration_->getEndJP().size());

    ASSERT_EQ(configuration_->getForceThresholdN(), 1);
    std::cout << configuration_->getMeasurementPerPoint() << std::endl;
    ASSERT_EQ(configuration_->getMeasurementPerPoint(), 10);
}

TEST_F(FtSensorCalibratorConfigShould, returnFalseAfterParsingBadConfigFile) {
    std::vector<std::string> filenames = {
        testDirName_ + "badConfigurrationMissingField.json",
        testDirName_ + "badConfigurationJsonCorrupted.json",
        testDirName_ + "badConfigurationJointsArray.json",
    };
    for (const auto& filename : filenames) {
        ASSERT_FALSE(configuration_->parse(filename));
    }
}
