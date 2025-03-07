/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *         Bartos Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include "DistanceMeasures/Rotation.hpp"
#include "EventLogger/EventLogger.hpp"

using crf::math::rotation::Rotation;

class RotationDistanceMeasureShould : public ::testing::Test {
 protected:
    RotationDistanceMeasureShould() :
        logger_("RotationDistanceMeasureShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~RotationDistanceMeasureShould() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
};

TEST_F(RotationDistanceMeasureShould, returnCorrectResults) {
    Rotation desiredRotation(Eigen::Quaterniond({0.644, 0.368, 0.644, 0.184}), 7.0e-4);
    Rotation currentRotation(Eigen::Quaterniond({0.644, 0.368, 0.644, 0.184}), 7.0e-4);

    Eigen::Vector<double, 3> cardanDistanceMeasure =
        crf::math::distancemeasures::byCardanXYZ(currentRotation, desiredRotation);
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(cardanDistanceMeasure[i], 0.0);
    }

    Eigen::Vector<double, 3> eulerDistanceMeasure =
        crf::math::distancemeasures::byEulerZXZ(currentRotation, desiredRotation);
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(eulerDistanceMeasure[i], 0.0);
    }

    Eigen::Vector<double, 3> axisAngleDistanceMeasure =
        crf::math::distancemeasures::byRotationMatrix(currentRotation, desiredRotation);
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(axisAngleDistanceMeasure[i], 0.0);
    }

    Eigen::Vector<double, 3> quaternionDistanceMeasure =
        crf::math::distancemeasures::byQuaternion(currentRotation, desiredRotation);
    for (int i = 0; i < 3; i++) {
        ASSERT_EQ(quaternionDistanceMeasure[i], 0.0);
    }
}

TEST_F(RotationDistanceMeasureShould, returnCorrectResultsDifferentFrom0) {
    Rotation desiredRotation(crf::math::rotation::CardanXYZ({0.1745329, 0.0872665, -0.1745329}));
    Rotation currentRotation(crf::math::rotation::CardanXYZ({0, 0, 0}));

    Eigen::Vector<double, 3> cardanDistanceMeasure =
        crf::math::distancemeasures::byCardanXYZ(currentRotation, desiredRotation);
    ASSERT_NEAR(cardanDistanceMeasure(0), 0.17453, 0.00001);
    ASSERT_NEAR(cardanDistanceMeasure(1), 0.08726, 0.00001);
    ASSERT_NEAR(cardanDistanceMeasure(2), -0.17453, 0.00001);

    Eigen::Vector<double, 3> eulerDistanceMeasure =
        crf::math::distancemeasures::byEulerZXZ(currentRotation, desiredRotation);
    ASSERT_NEAR(eulerDistanceMeasure(0), -0.46670, 0.00001);
    ASSERT_NEAR(eulerDistanceMeasure(1), 0.19494, 0.00001);
    ASSERT_NEAR(eulerDistanceMeasure(2), 0.28453, 0.00001);

    Eigen::Vector<double, 3> axisAngleDistanceMeasure =
        crf::math::distancemeasures::byRotationMatrix(currentRotation, desiredRotation);
    EXPECT_NEAR(axisAngleDistanceMeasure(0), 0.17945, 0.00001);
    EXPECT_NEAR(axisAngleDistanceMeasure(1), 0.07076, 0.00001);
    EXPECT_NEAR(axisAngleDistanceMeasure(2), -0.17945, 0.00001);

    Eigen::Vector<double, 3> quaternionDistanceMeasure =
        crf::math::distancemeasures::byQuaternion(currentRotation, desiredRotation);
    EXPECT_NEAR(quaternionDistanceMeasure(0), 0.09053, 0.00001);
    EXPECT_NEAR(quaternionDistanceMeasure(1), 0.03570, 0.00001);
    EXPECT_NEAR(quaternionDistanceMeasure(2), -0.09053, 0.00001);
}
