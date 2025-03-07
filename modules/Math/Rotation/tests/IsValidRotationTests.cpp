/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/IsValidRotation.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::math::rotation::isUnitaryQuaternion;
using crf::math::rotation::isRotationMatrix;

class IsValidRotationShould : public ::testing::Test {
 protected:
    IsValidRotationShould() :
        unitaryQuaternion_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        nonUnitaryQuaternion12_(
            {0.5505666516802112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        nonUnitaryQuaternion5_(
            {0.5506666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        rotationMatrix_({{ 0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
                         {-0.0173036611331485, -0.1325610914209059,  0.9910237839490472},
                         {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        nonOrthogonalMatrix12_({{ 0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
                                {-0.0173036611331485, -0.1325610914209059,  0.9910237839490472},
                                {-0.9092974268266818, -0.4100917877109334, -0.0707312888348917}}),
        nonOrthogonalMatrix5_({{ 0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
                               {-0.0172036611331485, -0.1325610914209059,  0.9910237839490472},
                               {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        orthogonalNonDet1Matrix_(-rotationMatrix_),
        nonOrthogonalNonDet1Matrix12_(-nonOrthogonalMatrix12_),
        nonOrthogonalNonDet1Matrix5_(-nonOrthogonalMatrix5_) {
    }

    const Eigen::Quaterniond unitaryQuaternion_;
    const Eigen::Quaterniond nonUnitaryQuaternion12_;
    const Eigen::Quaterniond nonUnitaryQuaternion5_;

    const Eigen::Matrix3d rotationMatrix_;
    const Eigen::Matrix3d nonOrthogonalMatrix12_;
    const Eigen::Matrix3d nonOrthogonalMatrix5_;
    const Eigen::Matrix3d orthogonalNonDet1Matrix_;
    const Eigen::Matrix3d nonOrthogonalNonDet1Matrix12_;
    const Eigen::Matrix3d nonOrthogonalNonDet1Matrix5_;
};

TEST_F(IsValidRotationShould, detectUnitaryQuaternion) {
    ASSERT_TRUE(isUnitaryQuaternion(unitaryQuaternion_));
    ASSERT_TRUE(isUnitaryQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_TRUE(isUnitaryQuaternion(nonUnitaryQuaternion5_, 1e-4));
}

TEST_F(IsValidRotationShould, detectNonUnitaryQuaternion) {
    ASSERT_FALSE(isUnitaryQuaternion(nonUnitaryQuaternion12_, 1e-12));
    ASSERT_FALSE(isUnitaryQuaternion(nonUnitaryQuaternion5_, 1e-5));
}

TEST_F(IsValidRotationShould, detectRotationMatrix) {
    ASSERT_TRUE(isRotationMatrix(rotationMatrix_));
    ASSERT_TRUE(isRotationMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_TRUE(isRotationMatrix(nonOrthogonalMatrix5_, 1e-4));
}

TEST_F(IsValidRotationShould, detectNonOrthogonalMatrix) {
    ASSERT_FALSE(isRotationMatrix(nonOrthogonalMatrix12_, 1e-12));
    ASSERT_FALSE(isRotationMatrix(nonOrthogonalMatrix5_, 1e-5));
}

TEST_F(IsValidRotationShould, detectOrthogonalNonDet1Matrix) {
    ASSERT_FALSE(isRotationMatrix(orthogonalNonDet1Matrix_));
    ASSERT_FALSE(isRotationMatrix(nonOrthogonalNonDet1Matrix12_, 1e-11));
    ASSERT_FALSE(isRotationMatrix(nonOrthogonalNonDet1Matrix5_, 1e-4));
}

TEST_F(IsValidRotationShould, detectNonOrthogonalNonDet1Matrix) {
    ASSERT_FALSE(isRotationMatrix(orthogonalNonDet1Matrix_, 1e-16));
    ASSERT_FALSE(isRotationMatrix(nonOrthogonalNonDet1Matrix12_, 1e-12));
    ASSERT_FALSE(isRotationMatrix(nonOrthogonalNonDet1Matrix5_, 1e-5));
}
