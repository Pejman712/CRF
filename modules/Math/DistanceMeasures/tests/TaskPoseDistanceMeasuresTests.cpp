/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *         Bartos Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "DistanceMeasures/TaskPose.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::utility::types::TaskPose;

class TaskPoseDistanceMeasureShould : public ::testing::Test {
 protected:
    TaskPoseDistanceMeasureShould() :
        logger_("TaskPoseDistanceMeasureShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~TaskPoseDistanceMeasureShould() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
};

TEST_F(TaskPoseDistanceMeasureShould, returnCorrectResults) {
    TaskPose desiredPose(
        {0.2, 0.3, 0.4}, Eigen::Quaterniond({0.644, 0.368, 0.644, 0.184}), 7.0e-4);
    TaskPose currentPose(
        {0.2, 0.3, 0.4}, Eigen::Quaterniond({0.644, 0.368, 0.644, 0.184}), 7.0e-4);

    Eigen::Vector<double, 6> cardanDistanceMeasure =
        crf::math::distancemeasures::byCardanXYZ(currentPose, desiredPose);
    for (int i = 0; i < 6; i++) {
        ASSERT_EQ(cardanDistanceMeasure[i], 0.0);
    }

    Eigen::Vector<double, 6> eulerDistanceMeasure =
        crf::math::distancemeasures::byEulerZXZ(currentPose, desiredPose);
    for (int i = 0; i < 6; i++) {
        ASSERT_EQ(eulerDistanceMeasure[i], 0.0);
    }

    Eigen::Vector<double, 6> axisAngleDistanceMeasure =
        crf::math::distancemeasures::byRotationMatrix(currentPose, desiredPose);
    for (int i = 0; i < 6; i++) {
        ASSERT_EQ(axisAngleDistanceMeasure[i], 0.0);
    }

    Eigen::Vector<double, 6> quaternionDistanceMeasure =
        crf::math::distancemeasures::byQuaternion(currentPose, desiredPose);
    for (int i = 0; i < 6; i++) {
        ASSERT_EQ(quaternionDistanceMeasure[i], 0.0);
    }
}

TEST_F(TaskPoseDistanceMeasureShould, returnCorrectResultsDifferentFrom0) {
    TaskPose desiredPose(
        {0.2, 0.3, 0.4},
        crf::math::rotation::CardanXYZ({0.1745329, 0.0872665, -0.1745329}));
    TaskPose currentPose({0, 0, 0}, crf::math::rotation::CardanXYZ({0, 0, 0}));

    Eigen::Vector<double, 6> cardanDistanceMeasure =
        crf::math::distancemeasures::byCardanXYZ(currentPose, desiredPose);
    ASSERT_NEAR(cardanDistanceMeasure(0), 0.2, 0.01);
    ASSERT_NEAR(cardanDistanceMeasure(1), 0.3, 0.01);
    ASSERT_NEAR(cardanDistanceMeasure(2), 0.4, 0.01);
    ASSERT_NEAR(cardanDistanceMeasure(3), 0.17453, 0.00001);
    ASSERT_NEAR(cardanDistanceMeasure(4), 0.08726, 0.00001);
    ASSERT_NEAR(cardanDistanceMeasure(5), -0.17453, 0.00001);

    Eigen::Vector<double, 6> eulerDistanceMeasure =
        crf::math::distancemeasures::byEulerZXZ(currentPose, desiredPose);
    ASSERT_NEAR(eulerDistanceMeasure(0), 0.2, 0.01);
    ASSERT_NEAR(eulerDistanceMeasure(1), 0.3, 0.01);
    ASSERT_NEAR(eulerDistanceMeasure(2), 0.4, 0.01);
    ASSERT_NEAR(eulerDistanceMeasure(3), -0.46670, 0.00001);
    ASSERT_NEAR(eulerDistanceMeasure(4), 0.19494, 0.00001);
    ASSERT_NEAR(eulerDistanceMeasure(5), 0.28453, 0.00001);

    Eigen::Vector<double, 6> axisAngleDistanceMeasure =
        crf::math::distancemeasures::byRotationMatrix(currentPose, desiredPose);
    EXPECT_NEAR(axisAngleDistanceMeasure(0), 0.2, 0.01);
    EXPECT_NEAR(axisAngleDistanceMeasure(1), 0.3, 0.01);
    EXPECT_NEAR(axisAngleDistanceMeasure(2), 0.4, 0.01);
    EXPECT_NEAR(axisAngleDistanceMeasure(3), 0.17945, 0.00001);
    EXPECT_NEAR(axisAngleDistanceMeasure(4), 0.07076, 0.00001);
    EXPECT_NEAR(axisAngleDistanceMeasure(5), -0.17945, 0.00001);

    Eigen::Vector<double, 6> quaternionDistanceMeasure =
        crf::math::distancemeasures::byQuaternion(currentPose, desiredPose);
    EXPECT_NEAR(quaternionDistanceMeasure(0), 0.2, 0.01);
    EXPECT_NEAR(quaternionDistanceMeasure(1), 0.3, 0.01);
    EXPECT_NEAR(quaternionDistanceMeasure(2), 0.4, 0.01);
    EXPECT_NEAR(quaternionDistanceMeasure(3), 0.09053, 0.00001);
    EXPECT_NEAR(quaternionDistanceMeasure(4), 0.03570, 0.00001);
    EXPECT_NEAR(quaternionDistanceMeasure(5), -0.09053, 0.00001);
}
