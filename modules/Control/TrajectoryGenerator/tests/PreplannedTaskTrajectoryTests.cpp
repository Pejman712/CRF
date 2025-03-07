/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <vector>

#include "TrajectoryGenerator/PreplannedTaskTrajectory/PreplannedTaskTrajectory.hpp"

using crf::control::trajectorygenerator::PreplannedTaskTrajectory;

class PreplannedTaskTrajectoryShould: public ::testing::Test {
 protected:
    PreplannedTaskTrajectoryShould():
        logger_("PreplannedTaskTrajectoryShould"),
        filePath_(__FILE__),
        cycleTime_(0.01) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        filePath_ = filePath_.substr(0, filePath_.find("PreplannedTaskTrajectoryTests.cpp"));
        filePath_ += "csvTraj/";
    }
    ~PreplannedTaskTrajectoryShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::string filePath_;
    double cycleTime_;
    std::unique_ptr<PreplannedTaskTrajectory> sut_;
};

TEST_F(PreplannedTaskTrajectoryShould, WrongInputCycleTime) {
    double cycleTime = -0.3;
    ASSERT_THROW(sut_.reset(new PreplannedTaskTrajectory(
        filePath_ + "CorrectFile7PoseCardanVelocityAndAcceleration.csv", cycleTime)),
        std::invalid_argument);
}

TEST_F(PreplannedTaskTrajectoryShould, WrongInputCSVFile) {
    std::vector<std::string> wrongCSVFiles({
        "NonExistentFile.csv",
        "WrongFile1EmptyFile.csv",
        "WrongFile2WrongTitlesLocation.csv",
        "WrongFile3WrongOrientationTitlesLocation.csv",
        "WrongFile4WrongCustomTitlesLocation.csv",
        "WrongFile5WrongTitles.csv",
        "WrongFile6WrongTitlesCustomAnd6DTaskSpaceMixed.csv",
        "WrongFile7MismatchNumberOfElements.csv",
        "WrongFile8MismatchNumberOfElements2ndLine.csv"
    });
    for (uint64_t i = 0; i < wrongCSVFiles.size(); i++) {
        ASSERT_THROW(sut_.reset(new PreplannedTaskTrajectory(
            filePath_ + wrongCSVFiles[i], cycleTime_)), std::invalid_argument);
    }
}

TEST_F(PreplannedTaskTrajectoryShould, TestCorrectConstructor) {
    std::vector<std::string> correctCSVFiles({
        "CorrectFile1PoseCardan.csv",
        "CorrectFile2PoseEulerZXZ.csv",
        "CorrectFile3PoseAxisAngle.csv",
        "CorrectFile4PoseQuaternion.csv",
        "CorrectFile5Velocity.csv",
        "CorrectFile6Acceleration.csv",
        "CorrectFile7PoseCardanVelocityAndAcceleration.csv"
    });
    for (uint64_t i = 0; i < correctCSVFiles.size(); i++) {
        ASSERT_NO_THROW(sut_.reset(new PreplannedTaskTrajectory(
            filePath_ + correctCSVFiles[i], cycleTime_)));
    }
}

TEST_F(PreplannedTaskTrajectoryShould, TestNotAvailableFunctions) {
    ASSERT_NO_THROW(sut_.reset(new PreplannedTaskTrajectory(
        filePath_ + "CorrectFile7PoseCardanVelocityAndAcceleration.csv", cycleTime_)));

    ASSERT_THROW(sut_->setProfileVelocity(crf::utility::types::TaskVelocity()),
        std::runtime_error);
    ASSERT_THROW(sut_->setProfileAcceleration(crf::utility::types::TaskAcceleration()),
        std::runtime_error);
}

TEST_F(PreplannedTaskTrajectoryShould, TestErrorsGetTrajectoryPointFunction) {
    ASSERT_NO_THROW(sut_.reset(new PreplannedTaskTrajectory(
        filePath_ + "CorrectFile7PoseCardanVelocityAndAcceleration.csv", cycleTime_)));

    // Time requested is not following the correct chronology
    /*ASSERT_THROW(sut_->getTrajectoryPoint(0.02), std::runtime_error);*/
}

TEST_F(PreplannedTaskTrajectoryShould, TestGoodBehaviorOfTheOtherFunctions) {
    ASSERT_NO_THROW(sut_.reset(new PreplannedTaskTrajectory(
        filePath_ + "CorrectFile7PoseCardanVelocityAndAcceleration.csv", cycleTime_)));
    // Check good behavior getTrajectoryPoint() function
    crf::utility::types::TaskSignals result;
    std::vector<double> values({0, 1, 2, 3, 4, 5});
    double value = values[0];
    double time = 0.0;
    for (uint64_t i = 0; i < (values.size() + 3); i++) {
        // Check good behavior isTrajectoryRunning() function
        if (i < values.size()) {
            // Inside the trajectory
            ASSERT_TRUE(sut_->isTrajectoryRunning());
            value = i;
        } else {
            // Outside the trajectory
            ASSERT_FALSE(sut_->isTrajectoryRunning());
            value = values.size() - 1;
        }
        ASSERT_NO_THROW(result = sut_->getTrajectoryPoint(time));
        // TaskPose
        ASSERT_DOUBLE_EQ(
            result.pose.value().getPosition()(0),
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getPosition()(1),
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getPosition()(2),
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getCardanXYZ()[0],
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getCardanXYZ()[1],
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getCardanXYZ()[2],
            values[value]);
        // TaskVelocity
        ASSERT_DOUBLE_EQ(result.velocity.value()[0], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[1], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[2], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[3], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[4], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[5], values[value]);
        // TaskAcceleration
        ASSERT_DOUBLE_EQ(result.acceleration.value()[0], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[1], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[2], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[3], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[4], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[5], values[value]);
        time += cycleTime_;
    }

    // Trying to restart the trajectory without using reset() function
    time = 0.0;
    /*ASSERT_THROW(result = sut_->getTrajectoryPoint(time), std::runtime_error);*/

    // Affter using reset() function the trajectory can be restarted
    ASSERT_NO_THROW(sut_->reset());
    for (uint64_t i = 0; i < (values.size() + 3); i++) {
        // Check good behavior isTrajectoryRunning() function
        if (i < values.size()) {
            // Inside the trajectory
            ASSERT_TRUE(sut_->isTrajectoryRunning());
            value = i;
        } else {
            // Outside the trajectory
            ASSERT_FALSE(sut_->isTrajectoryRunning());
            value = values.size() - 1;
        }
        ASSERT_NO_THROW(result = sut_->getTrajectoryPoint(time));
        // TaskPose
        ASSERT_DOUBLE_EQ(
            result.pose.value().getPosition()(0),
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getPosition()(1),
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getPosition()(2),
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getCardanXYZ()[0],
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getCardanXYZ()[1],
            values[value]);
        ASSERT_DOUBLE_EQ(
            result.pose.value().getCardanXYZ()[2],
            values[value]);
        // TaskVelocity
        ASSERT_DOUBLE_EQ(result.velocity.value()[0], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[1], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[2], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[3], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[4], values[value]);
        ASSERT_DOUBLE_EQ(result.velocity.value()[5], values[value]);
        // TaskAcceleration
        ASSERT_DOUBLE_EQ(result.acceleration.value()[0], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[1], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[2], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[3], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[4], values[value]);
        ASSERT_DOUBLE_EQ(result.acceleration.value()[5], values[value]);
        time += cycleTime_;
    }
}
