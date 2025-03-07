/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "LinearStage/LinearStageConfiguration.hpp"

using testing::_;
using testing::Return;

using crf::actuators::linearstage::LinearStageConfiguration;

class LinearStageConfigurationShould: public ::testing::Test {
 protected:
    LinearStageConfigurationShould():
    logger_("LinearStageConfigurationShould") {
        configuration_ = std::make_unique<LinearStageConfiguration>();
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() override {
        goodConfiguration_["rotationToLinearRatio"] = 234.5;
        goodConfiguration_["minimumPosition"] = 0;
        goodConfiguration_["maximumPosition"] = 23;
        goodConfiguration_["maximumVelocity"] = 54;
        goodConfiguration_["maximumAcceleration"] = 20;
        goodConfiguration_["maximumDeceleration"] = 21;

        goodConfigurationNoDeceleration_["rotationToLinearRatio"] = 234.5;
        goodConfigurationNoDeceleration_["minimumPosition"] = 0;
        goodConfigurationNoDeceleration_["maximumPosition"] = 23;
        goodConfigurationNoDeceleration_["maximumVelocity"] = 54;
        goodConfigurationNoDeceleration_["maximumAcceleration"] = 20;

        goodConfigurationNoLimits_["rotationToLinearRatio"] = 234.5;
        goodConfigurationNoLimits_["maximumVelocity"] = 54;
        goodConfigurationNoLimits_["maximumAcceleration"] = 20;

        badConfigurationMissingField_["rotationToLinearRatio"] = 234.5;
        badConfigurationMissingField_["maximumAcceleration"] = 20;
    }

    bool checkConfigEmptiness() {
        return ((configuration_->getLinearToRotationRatio() == 0) &&
                (configuration_->getMaximumAcceleration() == 0) &&
                (configuration_->getMaximumDeceleration() == 0) &&
                (configuration_->getMaximumVelocity() == 0) &&
                (configuration_->getMinimumPosition() == 0) &&
                (configuration_->getMaximumPosition() == 0) &&
                (configuration_->getRotationToLinearRatio() == 0));
    }

    ~LinearStageConfigurationShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<LinearStageConfiguration> configuration_;

    nlohmann::json goodConfiguration_;
    nlohmann::json goodConfigurationNoDeceleration_;
    nlohmann::json goodConfigurationNoLimits_;
    nlohmann::json badConfigurationMissingField_;
};

TEST_F(LinearStageConfigurationShould, badConfigurationReturnsFalse) {
    configuration_.reset(new LinearStageConfiguration());
    ASSERT_FALSE(configuration_->parse(badConfigurationMissingField_));
}

TEST_F(LinearStageConfigurationShould, correctlyParseGoodConfiguration) {
    configuration_.reset(new LinearStageConfiguration());
    ASSERT_TRUE(configuration_->parse(goodConfiguration_));

    ASSERT_NEAR(configuration_->getRotationToLinearRatio(), 234.5, 1e-3);
    ASSERT_NEAR(configuration_->getLinearToRotationRatio(), 1/234.5, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumAcceleration(), 20, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumVelocity(), 54, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumDeceleration(), 21, 1e-3);
    ASSERT_TRUE(configuration_->hasMinimumPosition());
    ASSERT_NEAR(configuration_->getMinimumPosition(), 0, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumPosition(), 23, 1e-3);
    ASSERT_TRUE(configuration_->hasMaximumPosition());
}

TEST_F(LinearStageConfigurationShould, configurationIsClearedOnBadParse) {
    configuration_.reset(new LinearStageConfiguration());
    ASSERT_TRUE(configuration_->parse(goodConfiguration_));

    ASSERT_NEAR(configuration_->getRotationToLinearRatio(), 234.5, 1e-3);
    ASSERT_NEAR(configuration_->getLinearToRotationRatio(), 1/234.5, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumAcceleration(), 20, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumVelocity(), 54, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumDeceleration(), 21, 1e-3);
    ASSERT_TRUE(configuration_->hasMinimumPosition());
    ASSERT_NEAR(configuration_->getMinimumPosition(), 0, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumPosition(), 23, 1e-3);
    ASSERT_TRUE(configuration_->hasMaximumPosition());

    ASSERT_FALSE(configuration_->parse(badConfigurationMissingField_));
    ASSERT_TRUE(checkConfigEmptiness());
}

TEST_F(LinearStageConfigurationShould, corectlyUseSameAcceleration) {
    configuration_.reset(new LinearStageConfiguration());
    ASSERT_TRUE(configuration_->parse(goodConfigurationNoDeceleration_));

    ASSERT_NEAR(configuration_->getRotationToLinearRatio(), 234.5, 1e-3);
    ASSERT_NEAR(configuration_->getLinearToRotationRatio(), 1/234.5, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumAcceleration(), 20, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumVelocity(), 54, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumDeceleration(), 20, 1e-3);
    ASSERT_TRUE(configuration_->hasMinimumPosition());
    ASSERT_NEAR(configuration_->getMinimumPosition(), 0, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumPosition(), 23, 1e-3);
    ASSERT_TRUE(configuration_->hasMaximumPosition());
}

TEST_F(LinearStageConfigurationShould, correctlyParseNoPositionLimits) {
    configuration_.reset(new LinearStageConfiguration());
    ASSERT_TRUE(configuration_->parse(goodConfigurationNoLimits_));

    ASSERT_NEAR(configuration_->getRotationToLinearRatio(), 234.5, 1e-3);
    ASSERT_NEAR(configuration_->getLinearToRotationRatio(), 1/234.5, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumAcceleration(), 20, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumVelocity(), 54, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumDeceleration(), 20, 1e-3);
    ASSERT_FALSE(configuration_->hasMinimumPosition());
    ASSERT_NEAR(configuration_->getMinimumPosition(), 0, 1e-3);
    ASSERT_NEAR(configuration_->getMaximumPosition(), 0, 1e-3);
    ASSERT_FALSE(configuration_->hasMaximumPosition());
}
