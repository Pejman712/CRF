/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/Comparison.hpp"
#include "Rotation/RotationClass.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::math::rotation::Rotation;
using crf::math::rotation::RotationRepresentation;
using crf::math::rotation::CardanXYZ;
using crf::math::rotation::EulerZXZ;
using crf::math::rotation::areAlmostEqual;

class RotationClassShould : public ::testing::Test {
 protected:
    RotationClassShould() :
        sut_(),
        inputQuaternion1_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        inputMatrix1_({{ 0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
                       {-0.0173036611331485, -0.1325610914209059,  0.9910237839490472},
                       {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        inputAngleAxis1_(
            1.9755068924749106,
            Eigen::Vector3d({-0.7621249848254679, 0.4328991822668132, 0.4814185346426796})),
        inputCardanXYZ1_({1.4, 2.0, 3.1}),
        inputEulerZXZ1_({1.1471123464639490, -1.6415867259093058, 0.1139727950424842}),
        inputQuaternion2_(
            {0.3232781413160111, 0.5504467585659593, 0.7697351635084868, 0.0027179753598398}),
        inputMatrix2_({{-0.1849992186629872,  0.8456391273700271,  0.5006693073825711},
                       { 0.8491537754599140,  0.3940019571883435, -0.3517105675892164},
                       {-0.4946849044758272,  0.3600790524212897, -0.7909677119144169}}),
        inputAngleAxis2_(
            2.4832094549611980,
            Eigen::Vector3d({0.5816806901502302, 0.8134121496309343, 0.0028722010957822})),
        inputCardanXYZ2_({2.7143908348394090, 0.5174722288733832, 1.7853074093465211}),
        inputEulerZXZ2_({-0.9415926535897930, 2.4831853071795869, 0.9584073464102065}),
        nonUnitaryQuaternion12_(
            {0.5505666516802112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        nonUnitaryQuaternion5_(
            {0.5506666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        nonOrthogonalMatrix12_({{ 0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
                                {-0.0173036611331485, -0.1325610914209059,  0.9910237839490472},
                                {-0.9092974268266818, -0.4100917877109334, -0.0707312888348917}}),
        nonOrthogonalMatrix5_({{ 0.4157869320692789, -0.9023592869214287, -0.1134413699981963},
                               {-0.0172036611331485, -0.1325610914209059,  0.9910237839490472},
                               {-0.9092974268256818, -0.4100917877109334, -0.0707312888348917}}),
        orthogonalNonDet1Matrix12_(-nonOrthogonalMatrix12_),
        orthogonalNonDet1Matrix5_(-nonOrthogonalMatrix5_),
        os_() {
    }

    void SetUp() override {
        sut_.reset(new Rotation());
        outputQuaternion_ = Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0});
        outputMatrix_ = Eigen::Matrix3d::Identity();
        outputAngleAxis_ = Eigen::AngleAxisd(0.0, Eigen::Vector3d({1.0, 0.0, 0.0}));
        outputCardanXYZ_ = CardanXYZ();
        outputEulerZXZ_ = EulerZXZ();
        os_ = std::stringstream();
    }

    std::unique_ptr<Rotation> sut_;
    const Eigen::Quaterniond inputQuaternion1_;
    const Eigen::Matrix3d inputMatrix1_;
    const Eigen::AngleAxisd inputAngleAxis1_;
    const CardanXYZ inputCardanXYZ1_;
    const EulerZXZ inputEulerZXZ1_;

    const Eigen::Quaterniond inputQuaternion2_;
    const Eigen::Matrix3d inputMatrix2_;
    const Eigen::AngleAxisd inputAngleAxis2_;
    const CardanXYZ inputCardanXYZ2_;
    const EulerZXZ inputEulerZXZ2_;

    const Eigen::Quaterniond nonUnitaryQuaternion12_;
    const Eigen::Quaterniond nonUnitaryQuaternion5_;

    const Eigen::Matrix3d nonOrthogonalMatrix12_;
    const Eigen::Matrix3d nonOrthogonalMatrix5_;
    const Eigen::Matrix3d orthogonalNonDet1Matrix12_;
    const Eigen::Matrix3d orthogonalNonDet1Matrix5_;

    Eigen::Quaterniond outputQuaternion_;
    Eigen::Matrix3d outputMatrix_;
    Eigen::AngleAxisd outputAngleAxis_;
    CardanXYZ outputCardanXYZ_;
    EulerZXZ outputEulerZXZ_;

    std::stringstream os_;
};

TEST_F(RotationClassShould, HaveCorrectDefaultConstructor) {
    ASSERT_NO_THROW(sut_.reset(new Rotation()));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(outputQuaternion_, Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0})));
}

TEST_F(RotationClassShould, HaveCorrectQuaternionConstructor) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion1_, outputQuaternion_));
    ASSERT_THROW(sut_.reset(new Rotation(nonUnitaryQuaternion12_)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new Rotation(nonUnitaryQuaternion12_, 1e-12)), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(nonUnitaryQuaternion12_, 1e-11)));
    ASSERT_THROW(sut_.reset(new Rotation(nonUnitaryQuaternion5_, 1e-5)), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(nonUnitaryQuaternion5_, 1e-4)));
}

TEST_F(RotationClassShould, HaveCorrectMatrixConstructor) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix1_, outputMatrix_));
    ASSERT_THROW(sut_.reset(new Rotation(nonOrthogonalMatrix12_)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new Rotation(nonOrthogonalMatrix12_, 1e-12)), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(nonOrthogonalMatrix12_, 1e-11)));
    ASSERT_THROW(sut_.reset(new Rotation(nonOrthogonalMatrix5_, 1e-5)), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(nonOrthogonalMatrix5_, 1e-4)));
    ASSERT_THROW(sut_.reset(new Rotation(orthogonalNonDet1Matrix12_)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new Rotation(orthogonalNonDet1Matrix12_, 1e-12)), std::invalid_argument);
    ASSERT_THROW(
        sut_.reset(new Rotation(orthogonalNonDet1Matrix12_, 1e-11)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new Rotation(orthogonalNonDet1Matrix5_, 1e-5)), std::invalid_argument);
    ASSERT_THROW(sut_.reset(new Rotation(orthogonalNonDet1Matrix5_, 1e-4)), std::invalid_argument);
}

TEST_F(RotationClassShould, HaveCorrectAngleAxisConstructor) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis1_, outputAngleAxis_));
}

TEST_F(RotationClassShould, HaveCorrectCardanXYZConstructor) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ1_, outputCardanXYZ_));
}

TEST_F(RotationClassShould, HaveCorrectEulerZXZConstructor) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ1_, outputEulerZXZ_));
}

TEST_F(RotationClassShould, HaveCorrectQuaternionSetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_), std::invalid_argument);
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion12_, 1e-11));
    ASSERT_NO_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-4));
    ASSERT_THROW(sut_->setQuaternion(nonUnitaryQuaternion5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion1_));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
}

TEST_F(RotationClassShould, HaveCorrectMatrixSetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_), std::invalid_argument);
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-12), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix12_, 1e-11));
    ASSERT_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-5), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setMatrix(nonOrthogonalMatrix5_, 1e-4));
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_), std::invalid_argument);
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-12), std::invalid_argument);
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix12_, 1e-11), std::invalid_argument);
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-5), std::invalid_argument);
    ASSERT_THROW(sut_->setMatrix(orthogonalNonDet1Matrix5_, 1e-4), std::invalid_argument);
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix1_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));
}

TEST_F(RotationClassShould, HaveCorrectAngleAxisSetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis1_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));
}

TEST_F(RotationClassShould, HaveCorrectCardanXYZSetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ1_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));
}

TEST_F(RotationClassShould, HaveCorrectEulerZXZSetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ1_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));
}

TEST_F(RotationClassShould, HaveCorrectQuaternionGetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputQuaternion_ = sut_->getQuaternion();
    ASSERT_TRUE(areAlmostEqual(inputQuaternion2_, outputQuaternion_));
}

TEST_F(RotationClassShould, HaveCorrectMatrixGetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputMatrix_ = sut_->getMatrix();
    ASSERT_TRUE(areAlmostEqual(inputMatrix2_, outputMatrix_));
}

TEST_F(RotationClassShould, HaveCorrectAngleAxisGetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputAngleAxis_ = sut_->getAngleAxis();
    ASSERT_TRUE(areAlmostEqual(inputAngleAxis2_, outputAngleAxis_));
}

TEST_F(RotationClassShould, HaveCorrectCardanXYZGetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputCardanXYZ_ = sut_->getCardanXYZ();
    ASSERT_TRUE(areAlmostEqual(inputCardanXYZ2_, outputCardanXYZ_));
}

TEST_F(RotationClassShould, HaveCorrectEulerZXZGetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    EXPECT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    outputEulerZXZ_ = sut_->getEulerZXZ();
    ASSERT_TRUE(areAlmostEqual(inputEulerZXZ2_, outputEulerZXZ_));
}

TEST_F(RotationClassShould, HaveCorrectRepresentationGetter) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setQuaternion(inputQuaternion2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setMatrix(inputMatrix2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setAngleAxis(inputAngleAxis2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setCardanXYZ(inputCardanXYZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Quaternion);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::Matrix);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::AngleAxis);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::CardanXYZ);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);

    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
    ASSERT_NO_THROW(sut_->setEulerZXZ(inputEulerZXZ2_));
    ASSERT_EQ(sut_->getRepresentation(), RotationRepresentation::EulerZXZ);
}

TEST_F(RotationClassShould, HaveCorrectStreamOperator) {
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputQuaternion1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Quaternion:\n"
        "w:  0.550566651690211\n"
        "x: -0.636215237228148\n"
        "y:  0.361380431590003\n"
        "z:  0.401883960402980");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputMatrix1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Matrix:\n"
        " 0.415786932069279 -0.902359286921429 -0.113441369998196\n"
        "-0.017303661133149 -0.132561091420906  0.991023783949047\n"
        "-0.909297426825682 -0.410091787710933 -0.070731288834892");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputAngleAxis1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "Angle:\n"
        "1.975506892474911\n"
        "Axis:\n"
        "-0.762124984825468\n"
        " 0.432899182266813\n"
        " 0.481418534642680");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputCardanXYZ1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "CardanXYZ:\n"
        "X: 1.400000000000000\n"
        "Y: 2.000000000000000\n"
        "Z: 3.100000000000000");
    os_ = std::stringstream();
    ASSERT_NO_THROW(sut_.reset(new Rotation(inputEulerZXZ1_)));
    ASSERT_NO_THROW(os_ << *sut_);
    ASSERT_EQ(
        os_.str(),
        "EulerZXZ:\n"
        "Z:  1.147112346463949\n"
        "X: -1.641586725909306\n"
        "Z:  0.113972795042484");
}
