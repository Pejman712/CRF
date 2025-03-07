/* © Copyright CERN 2023. All rights reserved. This software is released under a
 * CERN proprietary software license. Any permission to use it shall be granted
 * in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO 2023
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <gtest/gtest.h>

#include "KinematicChain/URDFKinematicChain/URDFKinematicChain.hpp"

Eigen::Vector4d quaterniondSubstractToVector(Eigen::Quaterniond q1, Eigen::Quaterniond q2);

class URDFKinematicChainShould : public ::testing::Test {
 protected:
    URDFKinematicChainShould() :
        logger_("URDFKinematicChainShould"),
        testDirName_(__FILE__),
        pathToURDFs_(testDirName_.substr(0, testDirName_.find("KinematicChain"))) {
        pathToURDFs_ += "KinematicChain/tests/config";
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~URDFKinematicChainShould() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::string pathToURDFs_;
    std::unique_ptr<crf::math::kinematicchain::URDFKinematicChain> sut_;

    const double PREC = 1e-14;
};

TEST_F(URDFKinematicChainShould, returnURDFIsNonExistent) {
    std::string pathToURDF("IDontExist");
    std::string eeName("leafLink");

    EXPECT_THROW(
        sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(pathToURDF, eeName, "")),
        std::invalid_argument);

    pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";
    EXPECT_THROW(
        sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
            pathToURDF, eeName, "nonExistentTool")),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, returnEndEffectorIsNonExistent) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";
    std::string eeName("IDontExist");

    EXPECT_THROW(
        sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(pathToURDF, eeName, "")),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, returnCorrectCustomParameters) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";
    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));

    crf::math::kinematicchain::DescriptionType out00 = sut_->getType();
    EXPECT_TRUE(out00 == crf::math::kinematicchain::DescriptionType::Combined);

    pathToURDF = pathToURDFs_ + "/KinematicChainArmTesting.urdf";
    std::string KinChainTestingEndEffectorPoint = "leafLink";

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, KinChainTestingEndEffectorPoint, ""));

    crf::math::kinematicchain::DescriptionType out01 = sut_->getType();
    EXPECT_TRUE(out01 == crf::math::kinematicchain::DescriptionType::Arm);

    pathToURDF = pathToURDFs_ + "/KinematicChainPlatformTestingBasedOnSPS.urdf";
    KinChainTestingEndEffectorPoint = "platform_root_link";

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, KinChainTestingEndEffectorPoint, ""));

    crf::math::kinematicchain::DescriptionType out03 = sut_->getType();
    EXPECT_TRUE(out03 == crf::math::kinematicchain::DescriptionType::Platform);
}

TEST_F(URDFKinematicChainShould, returnZeroAxisPresentPlatform) {
    std::string pathToURDF =
        pathToURDFs_ + "/KinematicChainPlatformTestingBasedOnSPSZeroAxisBase.urdf";
    std::string KinChainTestingEndEffectorPoint("platform_root_link");

    EXPECT_THROW(
        sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
            pathToURDF, KinChainTestingEndEffectorPoint, "")),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, returnNonYAxisPresentPlatform) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainPlatformTestingBasedOnSPSNonYAxis.urdf";
    std::string KinChainTestingEndEffectorPoint("platform_root_link");

    EXPECT_THROW(
        sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
            pathToURDF, KinChainTestingEndEffectorPoint, "")),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, throwFoundTooLessCorrectlyNamedWheels) {
    std::string pathToURDF = pathToURDFs_ +
        "/KinematicChainCombinedTestingBasedOnSPSTooLessCorr"
        "ectlyNamedWheels.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");

    EXPECT_THROW(
        sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
            pathToURDF, SPSRobotEndEffectorPoint, "")),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, throwInvalidWheelIndex) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));
    EXPECT_THROW(sut_->getAxis(crf::math::kinematicchain::Axes::IW, -1), std::invalid_argument);
    EXPECT_THROW(sut_->getAxis(crf::math::kinematicchain::Axes::IW, 4), std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, returnCorrectBaseParameters) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));

    double out00 = sut_->getWheelRadius();
    EXPECT_TRUE(out00);
    EXPECT_DOUBLE_EQ(out00, 0.076);

    double out01 = sut_->getPlatformL();
    EXPECT_TRUE(out01);
    EXPECT_DOUBLE_EQ(out01, 0.366);

    double out02 = sut_->getPlatformW();
    EXPECT_TRUE(out02);
    EXPECT_DOUBLE_EQ(out02, 0.278);

    Eigen::Vector3d out03 = sut_->getAxis(crf::math::kinematicchain::Axes::IW, 0);
    EXPECT_TRUE((out03 - Eigen::Vector3d(0, 1, 0)).norm() < PREC);

    Eigen::Vector3d out04 = sut_->getAxis(crf::math::kinematicchain::Axes::IW, 1);
    EXPECT_TRUE((out04 - Eigen::Vector3d(0, 1, 0)).norm() < PREC);

    Eigen::Vector3d out05 = sut_->getAxis(crf::math::kinematicchain::Axes::IW, 2);
    EXPECT_TRUE((out05 - Eigen::Vector3d(0, 1, 0)).norm() < PREC);

    Eigen::Vector3d out06 = sut_->getAxis(crf::math::kinematicchain::Axes::IW, 3);
    EXPECT_TRUE((out06 - Eigen::Vector3d(0, 1, 0)).norm() < PREC);
}

TEST_F(URDFKinematicChainShould, returnNoBasePresent) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainArmTesting.urdf";

    std::string KinematicChainArmTestingEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, KinematicChainArmTestingEndEffectorPoint, ""));

    EXPECT_THROW(sut_->getWheelRadius(), std::runtime_error);

    EXPECT_THROW(sut_->getPlatformL(), std::runtime_error);

    EXPECT_THROW(sut_->getPlatformW(), std::runtime_error);

    EXPECT_THROW(sut_->getAxis(crf::math::kinematicchain::Axes::IW, 1), std::runtime_error);
}

TEST_F(URDFKinematicChainShould, returnZeroAxisPresentArm) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainArmTestingZeroAxisArm.urdf";
    std::string KinChainTestingEndEffectorPoint("leafLink");

    EXPECT_THROW(
        sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
            pathToURDF, KinChainTestingEndEffectorPoint, "")),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, returnInvalidJointPositionsDimension) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";
    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));

    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::IJ, 0, crf::utility::types::JointPositions(11)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::IJ, 3, crf::utility::types::JointPositions(3)),
        std::invalid_argument);

    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IJE,
            1,
            crf::utility::types::JointPositions(11)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IJE,
            4,
            crf::utility::types::JointPositions(3)),
        std::invalid_argument);

    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIJ, 2, crf::utility::types::JointPositions(11)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIJ, 5, crf::utility::types::JointPositions(3)),
        std::invalid_argument);

    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE,
            0,
            crf::utility::types::JointPositions(11)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE,
            0,
            crf::utility::types::JointPositions(3)),
        std::invalid_argument);

    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIE, 0, crf::utility::types::JointPositions(11)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIE, 0, crf::utility::types::JointPositions(3)),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, returnInvalidJointIndex) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";
    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));

    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::IJ, 7, crf::utility::types::JointPositions(6)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::IJ, -1, crf::utility::types::JointPositions(6)),
        std::invalid_argument);

    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IJE,
            8,
            crf::utility::types::JointPositions(6)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IJE,
            -4,
            crf::utility::types::JointPositions(6)),
        std::invalid_argument);

    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIJ, 8, crf::utility::types::JointPositions(6)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIJ, -4, crf::utility::types::JointPositions(6)),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, returnInvalidWheelIndex) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainArmTesting.urdf";
    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));

    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::IW, 7, crf::utility::types::JointPositions(6)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::WW, -1, crf::utility::types::JointPositions(6)),
        std::invalid_argument);

    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIW,
            8,
            crf::utility::types::JointPositions(6)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIW,
            -4,
            crf::utility::types::JointPositions(6)),
        std::invalid_argument);

    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIW, 5, crf::utility::types::JointPositions(6)),
        std::invalid_argument);
    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIW, 0, crf::utility::types::JointPositions(6)),
        std::invalid_argument);
}

TEST_F(URDFKinematicChainShould, returnNoArmPresent) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainPlatformTestingBasedOnSPS.urdf";
    std::string SPSRobotEndEffectorPoint("platform_root_link");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));

    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::IJ, 0, crf::utility::types::JointPositions(4)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::IJ, 3, crf::utility::types::JointPositions(4)),
        std::runtime_error);

    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IJE,
            1,
            crf::utility::types::JointPositions(4)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IJE,
            4,
            crf::utility::types::JointPositions(4)),
        std::runtime_error);

    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIJ, 2, crf::utility::types::JointPositions(4)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIJ, 0, crf::utility::types::JointPositions(4)),
        std::runtime_error);

    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE,
            0,
            crf::utility::types::JointPositions(4)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE,
            0,
            crf::utility::types::JointPositions(4)),
        std::runtime_error);

    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIE, 0, crf::utility::types::JointPositions(4)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIE, 0, crf::utility::types::JointPositions(4)),
        std::runtime_error);
}

TEST_F(URDFKinematicChainShould, returnNoPlatformPresent) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainArmTesting.urdf";
    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));
    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::IW, 0, crf::utility::types::JointPositions(5)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeAxis(
            crf::math::kinematicchain::Axes::WW, 3, crf::utility::types::JointPositions(5)),
        std::runtime_error);

    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIW,
            1,
            crf::utility::types::JointPositions(5)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIW,
            4,
            crf::utility::types::JointPositions(5)),
        std::runtime_error);

    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIW, 2, crf::utility::types::JointPositions(5)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIW, 0, crf::utility::types::JointPositions(5)),
        std::runtime_error);

    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIW,
            0,
            crf::utility::types::JointPositions(5)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIW,
            0,
            crf::utility::types::JointPositions(5)),
        std::runtime_error);

    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIW, 0, crf::utility::types::JointPositions(5)),
        std::runtime_error);
    EXPECT_THROW(
        sut_->computeRotation(
            crf::math::kinematicchain::Rotations::IIW, 0, crf::utility::types::JointPositions(5)),
        std::runtime_error);
}

TEST_F(URDFKinematicChainShould, returnCorrectAxis) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));

    crf::utility::types::JointPositions jointPositions1(
        {-0.029005763708726,
         0.182452167505983,
         -1.565056014150725,
         -0.084539479817724,
         -3.029177341404146,
         -0.457014640871583,
         1.242448406390738,
         -94.155944487570736,
         0.933728162671238,
         0.350321001356112});
    Eigen::Vector3d out00 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 0, jointPositions1);
    EXPECT_TRUE((out00 - Eigen::Vector3d(0.0, 1.0, 0.0)).norm() < PREC);
    Eigen::Vector3d out01 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 1, jointPositions1);
    EXPECT_TRUE(
        (out01 - Eigen::Vector3d(0.000070915157858, -0.999999997360728, 0.000015798243149)).norm() <
        PREC);
    Eigen::Vector3d out02 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 2, jointPositions1);
    EXPECT_TRUE(
        (out02 - Eigen::Vector3d(0.971854469413412, 0.000065197431322, -0.235582015507277)).norm() <
        PREC);
    Eigen::Vector3d out03 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 3, jointPositions1);
    EXPECT_TRUE(
        (out03 - Eigen::Vector3d(0.07609499452041163, -0.9465758000821014, 0.3133748657883539))
            .norm() < PREC);
    Eigen::Vector3d out04 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 4, jointPositions1);
    EXPECT_TRUE(
        (out04 - Eigen::Vector3d(-0.790903992312942, 0.134014686456656, 0.597085369739851)).norm() <
        PREC);
    Eigen::Vector3d out05 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 5, jointPositions1);
    EXPECT_TRUE(
        (out05 - Eigen::Vector3d(0.299905643939044, 0.935373877861576, 0.187436158058435)).norm() <
        PREC);
}

TEST_F(URDFKinematicChainShould, returnCorrectAxisPositionChange) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));
    crf::utility::types::JointPositions jointPositions0(
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions jointPositions1(
        {-0.029005763708726,
         0.182452167505983,
         -1.565056014150725,
         -0.084539479817724,
         -3.029177341404146,
         -0.457014640871583,
         1.242448406390738,
         -94.155944487570736,
         0.933728162671238,
         0.350321001356112});
    crf::utility::types::JointPositions jointPositions2(
        {0.0,
         0.0,
         0.0,
         0.0,
         2.50376224635538,
         -2.39920164674269,
         3.06882035390508,
         0.251214937225332,
         1.30010048888839,
         3.13839840845743});
    crf::utility::types::JointPositions jointPositions3(
        {0.0,
         5.0,
         4.0,
         -2.0,
         -0.206916683399114,
         0.931158050055648,
         -2.98307931430608,
         2.150147559173,
         0.370912419318411,
         2.22487559854808});
    crf::utility::types::JointPositions jointPositions4(
        {0.0,
         0.0,
         0.0,
         0.0,
         -2.30007778521014,
         -2.9475078947703,
         2.75920871534614,
         -1.24843081561655,
         -1.28469880703987,
         -1.0496922993298});
    crf::utility::types::JointPositions jointPositions5(
        {-0.7581, 2.0783, -2.2220, 0.4488, 0.0446, -0.7562, 0.4243, -0.7939, 0.8598, 8.5562});

    Eigen::Vector3d out10 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 5, jointPositions5);
    EXPECT_TRUE(
        (out10 - Eigen::Vector3d(-0.040263151291035415, 0.85277779739251891, 0.52071960489545677))
            .norm() < PREC);
    Eigen::Vector3d out11 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 2, jointPositions1);
    EXPECT_TRUE(
        (out11 -
         Eigen::Vector3d(0.97185446941341169, 0.000065197431322466803, -0.23558201550727736))
            .norm() < PREC);
    Eigen::Vector3d out12 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 4, jointPositions2);
    EXPECT_TRUE(
        (out12 - Eigen::Vector3d(-0.097990040662644023, -0.41500580711949819, 0.90452646837338446))
            .norm() < PREC);
    Eigen::Vector3d out13 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 0, jointPositions3);
    EXPECT_TRUE((out13 - Eigen::Vector3d(0.0, 1.0, 0.0)).norm() < PREC);
    Eigen::Vector3d out14 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 3, jointPositions4);
    EXPECT_TRUE(
        (out14 - Eigen::Vector3d(0.28812628261714684, -0.37311824497321311, 0.88191043793195545))
            .norm() < PREC);
    Eigen::Vector3d out15 =
        sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 1, jointPositions0);
    EXPECT_TRUE(
        (out15 -
         Eigen::Vector3d(-0.000068695318673380428, -0.99999999736072809, -0.000023653695122292442))
            .norm() < PREC);

    std::srand(56461384);
    std::vector<double> jointPositionssRandom;

    jointPositionssRandom.push_back(-0.029005763708726);
    jointPositionssRandom.push_back(0.182452167505983);
    jointPositionssRandom.push_back(-1.565056014150725);
    jointPositionssRandom.push_back(-0.084539479817724);
    jointPositionssRandom.push_back(-3.029177341404146);
    jointPositionssRandom.push_back(-0.457014640871583);
    jointPositionssRandom.push_back(1.242448406390738);
    jointPositionssRandom.push_back(-94.155944487570736);
    jointPositionssRandom.push_back(0.933728162671238);
    jointPositionssRandom.push_back(0.350321001356112);

    for (int i = 0; i < 100; i++) {
        jointPositionssRandom = jointPositionssRandom;
        double random_variable = static_cast<double>(std::rand()) / RAND_MAX;
        for (int k = 0; k < 10; k++) {
            if (static_cast<double>(std::rand()) / RAND_MAX < random_variable) {
                jointPositionssRandom[k] += static_cast<double>(std::rand()) * 10 / RAND_MAX - 5;
            }
        }
        crf::utility::types::JointPositions jointPositionsIntermediate(jointPositionssRandom);
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE, 0, jointPositionsIntermediate);
    }

    out10 = sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 5, jointPositions5);
    EXPECT_TRUE(
        (out10 - Eigen::Vector3d(-0.040263151291035415, 0.85277779739251891, 0.52071960489545677))
            .norm() < PREC);
    out11 = sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 2, jointPositions1);
    EXPECT_TRUE(
        (out11 -
         Eigen::Vector3d(0.97185446941341169, 0.000065197431322466803, -0.23558201550727736))
            .norm() < PREC);
    out12 = sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 4, jointPositions2);
    EXPECT_TRUE(
        (out12 - Eigen::Vector3d(-0.097990040662644023, -0.41500580711949819, 0.90452646837338446))
            .norm() < PREC);
    out13 = sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 0, jointPositions3);
    EXPECT_TRUE((out13 - Eigen::Vector3d(0.0, 1.0, 0.0)).norm() < PREC);
    out14 = sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 3, jointPositions4);
    EXPECT_TRUE(
        (out14 - Eigen::Vector3d(0.28812628261714684, -0.37311824497321311, 0.88191043793195545))
            .norm() < PREC);
    out15 = sut_->computeAxis(crf::math::kinematicchain::Axes::IJ, 1, jointPositions0);
    EXPECT_TRUE(
        (out15 -
         Eigen::Vector3d(-0.000068695318673380428, -0.99999999736072809, -0.000023653695122292442))
            .norm() < PREC);
}

TEST_F(URDFKinematicChainShould, returnCorrectTranslation) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));

    crf::utility::types::JointPositions jointPositions3(
        {0.0,
         5.0,
         4.0,
         -2.0,
         -0.206916683399114,
         0.931158050055648,
         -2.98307931430608,
         2.150147559173,
         0.370912419318411,
         2.22487559854808});
    Eigen::Vector3d out20 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 5, jointPositions3);
    EXPECT_TRUE((out20 - Eigen::Vector3d(0, 0, 0)).norm() < PREC);
    Eigen::Vector3d out21 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 4, jointPositions3);
    EXPECT_TRUE(
        (out21 - Eigen::Vector3d(-0.012994816792144967, 0.06321083066549614, 0.028879328647261896))
            .norm() < PREC);
    Eigen::Vector3d out22 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 3, jointPositions3);
    EXPECT_TRUE(
        (out22 -
         Eigen::Vector3d(-0.031665720345178588, -0.027474159444249968, -0.01147695871057829))
            .norm() < PREC);
    Eigen::Vector3d out23 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 2, jointPositions3);
    EXPECT_TRUE(
        (out23 - Eigen::Vector3d(-2.4050587498632146, 1.773500321038108, 0.33076982200601712))
            .norm() < PREC);
    Eigen::Vector3d out24 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 1, jointPositions3);
    EXPECT_TRUE(
        (out24 - Eigen::Vector3d(-2.3851556559555211, 1.8084917705601056, 0.52692618707544492))
            .norm() < PREC);
    Eigen::Vector3d out25 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 0, jointPositions3);
    EXPECT_TRUE(
        (out25 - Eigen::Vector3d(-2.8055016575950518, 1.8084917705601047, 0.27579796861591088))
            .norm() < PREC);
    Eigen::Vector3d out26 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IIE, 0, jointPositions3);
    EXPECT_TRUE(
        (out26 - Eigen::Vector3d(-2.5553016575950518, 1.8097917705601048, 0.27579796861591088))
            .norm() < PREC);
}

TEST_F(URDFKinematicChainShould, returnCorrectTranslationPositionChange) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));
    crf::utility::types::JointPositions jointPositions0(
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions jointPositions1(
        {-0.029005763708726,
         0.182452167505983,
         -1.565056014150725,
         -0.084539479817724,
         -3.029177341404146,
         -0.457014640871583,
         1.242448406390738,
         -94.155944487570736,
         0.933728162671238,
         0.350321001356112});
    crf::utility::types::JointPositions jointPositions2(
        {0.0,
         0.0,
         0.0,
         0.0,
         2.50376224635538,
         -2.39920164674269,
         3.06882035390508,
         0.251214937225332,
         1.30010048888839,
         3.13839840845743});
    crf::utility::types::JointPositions jointPositions3(
        {0.0,
         5.0,
         4.0,
         -2.0,
         -0.206916683399114,
         0.931158050055648,
         -2.98307931430608,
         2.150147559173,
         0.370912419318411,
         2.22487559854808});
    crf::utility::types::JointPositions jointPositions4(
        {0.0,
         0.0,
         0.0,
         0.0,
         -2.30007778521014,
         -2.9475078947703,
         2.75920871534614,
         -1.24843081561655,
         -1.28469880703987,
         -1.0496922993298});
    crf::utility::types::JointPositions jointPositions5(
        {-0.7581, 2.0783, -2.2220, 0.4488, 0.0446, -0.7562, 0.4243, -0.7939, 0.8598, 8.5562});

    Eigen::Vector3d out30 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 3, jointPositions2);
    EXPECT_TRUE(
        (out30 -
         Eigen::Vector3d(-0.057090241244417278, -0.079404968550434718, -0.042616550727225123))
            .norm() < PREC);
    Eigen::Vector3d out31 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 5, jointPositions5);
    EXPECT_TRUE((out31 - Eigen::Vector3d(0.0, 0.0, 0.0)).norm() < PREC);
    Eigen::Vector3d out32 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 2, jointPositions3);
    EXPECT_TRUE(
        (out32 - Eigen::Vector3d(-2.4050587498632146, 1.773500321038108, 0.33076982200601712))
            .norm() < PREC);
    Eigen::Vector3d out33 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 4, jointPositions4);
    EXPECT_TRUE(
        (out33 -
         Eigen::Vector3d(-0.0095445368870074182, 0.042138059599550165, -0.055962270761624008))
            .norm() < PREC);
    Eigen::Vector3d out34 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 1, jointPositions1);
    EXPECT_TRUE(
        (out34 - Eigen::Vector3d(-7.6885946440131381, 88.199413527806513, -30.50572240970196))
            .norm() < PREC);
    Eigen::Vector3d out35 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 0, jointPositions0);
    EXPECT_TRUE(
        (out35 - Eigen::Vector3d(-0.030284594704167809, -1.337469030869699, -0.53923816280533932))
            .norm() < PREC);
    Eigen::Vector3d out36 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IIE, 0, jointPositions4);
    EXPECT_TRUE(
        (out36 - Eigen::Vector3d(0.84739529500138044, 1.5713712716656485, -0.57055161626035167))
            .norm() < PREC);

    std::srand(56461384);
    std::vector<double> jointPositionssRandom;

    jointPositionssRandom.push_back(-0.029005763708726);
    jointPositionssRandom.push_back(0.182452167505983);
    jointPositionssRandom.push_back(-1.565056014150725);
    jointPositionssRandom.push_back(-0.084539479817724);
    jointPositionssRandom.push_back(-3.029177341404146);
    jointPositionssRandom.push_back(-0.457014640871583);
    jointPositionssRandom.push_back(1.242448406390738);
    jointPositionssRandom.push_back(-94.155944487570736);
    jointPositionssRandom.push_back(0.933728162671238);
    jointPositionssRandom.push_back(0.350321001356112);

    for (int i = 0; i < 100; i++) {
        jointPositionssRandom = jointPositionssRandom;
        double random_variable = static_cast<double>(std::rand()) / RAND_MAX;
        for (int k = 0; k < 10; k++) {
            if (static_cast<double>(std::rand()) / RAND_MAX < random_variable) {
                jointPositionssRandom[k] += static_cast<double>(std::rand()) * 10 / RAND_MAX - 5;
            }
        }
        crf::utility::types::JointPositions jointPositionsIntermediate(jointPositionssRandom);
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE, 0, jointPositionsIntermediate);
    }
    out30 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 3, jointPositions2);
    EXPECT_TRUE(
        (out30 -
         Eigen::Vector3d(-0.057090241244417278, -0.079404968550434718, -0.042616550727225123))
            .norm() < PREC);
    out31 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 5, jointPositions5);
    EXPECT_TRUE((out31 - Eigen::Vector3d(0.0, 0.0, 0.0)).norm() < PREC);
    out32 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 2, jointPositions3);
    EXPECT_TRUE(
        (out32 - Eigen::Vector3d(-2.4050587498632146, 1.773500321038108, 0.33076982200601712))
            .norm() < PREC);
    out33 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 4, jointPositions4);
    EXPECT_TRUE(
        (out33 -
         Eigen::Vector3d(-0.0095445368870074182, 0.042138059599550165, -0.055962270761624008))
            .norm() < PREC);
    out34 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 1, jointPositions1);
    EXPECT_TRUE(
        (out34 - Eigen::Vector3d(-7.6885946440131381, 88.199413527806513, -30.50572240970196))
            .norm() < PREC);
    out35 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 0, jointPositions0);
    EXPECT_TRUE(
        (out35 - Eigen::Vector3d(-0.030284594704167809, -1.337469030869699, -0.53923816280533932))
            .norm() < PREC);
    out36 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IIE, 0, jointPositions4);
    EXPECT_TRUE(
        (out36 - Eigen::Vector3d(0.84739529500138044, 1.5713712716656485, -0.57055161626035167))
            .norm() < PREC);
}

TEST_F(URDFKinematicChainShould, returnCorrectTranslationPositionChangeFixedJointsAndTool) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainFixedJointsTest.urdf";

    std::string endEffectorName("leafLink");

    std::string toolPath = pathToURDFs_ + "/KinematicChainToolTest.urdf";

    sut_.reset(
        new crf::math::kinematicchain::URDFKinematicChain(pathToURDF, endEffectorName, toolPath));
    crf::utility::types::JointPositions jointPositions0({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions jointPositions1(
        {-3.029177341404146,
         -0.457014640871583,
         1.242448406390738,
         -94.155944487570736,
         0.933728162671238,
         0.350321001356112});
    crf::utility::types::JointPositions jointPositions2(
        {2.50376224635538,
         -2.39920164674269,
         3.06882035390508,
         0.251214937225332,
         1.30010048888839,
         3.13839840845743});
    crf::utility::types::JointPositions jointPositions3(
        {-0.206916683399114,
         0.931158050055648,
         -2.98307931430608,
         2.150147559173,
         0.370912419318411,
         2.22487559854808});
    crf::utility::types::JointPositions jointPositions4(
        {-2.30007778521014,
         -2.9475078947703,
         2.75920871534614,
         -1.24843081561655,
         -1.28469880703987,
         -1.0496922993298});
    crf::utility::types::JointPositions jointPositions5(
        {0.0446, -0.7562, 0.4243, -0.7939, 0.8598, 8.5562});

    Eigen::Vector3d out30 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 3, jointPositions2);
    EXPECT_TRUE(
        (out30 - Eigen::Vector3d(0.1923930026926749, 0.02321403325810106, -0.6734529529230414))
            .norm() < PREC);
    Eigen::Vector3d out31 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 5, jointPositions5);
    EXPECT_TRUE(
        (out31 - Eigen::Vector3d(-0.2602387667760826, 0.6537908094927911, -0.4080439178588813))
            .norm() < PREC);
    Eigen::Vector3d out32 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 2, jointPositions3);
    EXPECT_TRUE(
        (out32 - Eigen::Vector3d(-0.276625674043898, -0.7039122103325515, 0.2558259413519586))
            .norm() < PREC);
    Eigen::Vector3d out33 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 4, jointPositions4);
    EXPECT_TRUE(
        (out33 - Eigen::Vector3d(-0.2570933535835717, 0.8330081004014556, -0.02600733618554441))
            .norm() < PREC);
    Eigen::Vector3d out34 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 1, jointPositions1);
    EXPECT_TRUE(
        (out34 - Eigen::Vector3d(-0.2032959302526411, 0.05360705214512917, 1.382495562536721))
            .norm() < PREC);
    Eigen::Vector3d out35 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 0, jointPositions0);
    EXPECT_TRUE(
        (out35 - Eigen::Vector3d(-0.780004365665804, -0.3577942649955135, 0.8991883305193843))
            .norm() < PREC);
    Eigen::Vector3d out36 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IIE, 0, jointPositions4);
    EXPECT_TRUE(
        (out36 - Eigen::Vector3d(-0.1106218002710578, 0.7791524700068526, 0.1653345235030094))
            .norm() < PREC);

    std::srand(56461384);
    std::vector<double> jointPositionssRandom;

    jointPositionssRandom.push_back(-3.029177341404146);
    jointPositionssRandom.push_back(-0.457014640871583);
    jointPositionssRandom.push_back(1.242448406390738);
    jointPositionssRandom.push_back(-94.155944487570736);
    jointPositionssRandom.push_back(0.933728162671238);
    jointPositionssRandom.push_back(0.350321001356112);

    int chainSize = sut_->getChainSize();

    for (int i = 0; i < 100; i++) {
        jointPositionssRandom = jointPositionssRandom;
        double random_variable = static_cast<double>(std::rand()) / RAND_MAX;
        for (int k = 0; k < chainSize; k++) {
            if (static_cast<double>(std::rand()) / RAND_MAX < random_variable) {
                jointPositionssRandom[k] += static_cast<double>(std::rand()) * 10 / RAND_MAX - 5;
            }
        }
        crf::utility::types::JointPositions jointPositionsIntermediate(jointPositionssRandom);
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE, 0, jointPositionsIntermediate);
    }
    out30 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 3, jointPositions2);
    EXPECT_TRUE(
        (out30 - Eigen::Vector3d(0.1923930026926749, 0.02321403325810106, -0.6734529529230414))
            .norm() < PREC);
    out31 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 5, jointPositions5);
    EXPECT_TRUE(
        (out31 - Eigen::Vector3d(-0.2602387667760826, 0.6537908094927911, -0.4080439178588813))
            .norm() < PREC);
    out32 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 2, jointPositions3);
    EXPECT_TRUE(
        (out32 - Eigen::Vector3d(-0.276625674043898, -0.7039122103325515, 0.2558259413519586))
            .norm() < PREC);
    out33 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 4, jointPositions4);
    EXPECT_TRUE(
        (out33 - Eigen::Vector3d(-0.2570933535835717, 0.8330081004014556, -0.02600733618554441))
            .norm() < PREC);
    out34 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 1, jointPositions1);
    EXPECT_TRUE(
        (out34 - Eigen::Vector3d(-0.2032959302526411, 0.05360705214512917, 1.382495562536721))
            .norm() < PREC);
    out35 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IJE, 0, jointPositions0);
    EXPECT_TRUE(
        (out35 - Eigen::Vector3d(-0.780004365665804, -0.3577942649955135, 0.8991883305193843))
            .norm() < PREC);
    out36 =
        sut_->computeTranslation(crf::math::kinematicchain::Translations::IIE, 0, jointPositions4);
    EXPECT_TRUE(
        (out36 - Eigen::Vector3d(-0.1106218002710578, 0.7791524700068526, 0.1653345235030094))
            .norm() < PREC);
}

TEST_F(URDFKinematicChainShould, returnCorrectRotation) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");

    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));
    crf::utility::types::JointPositions jointPositions2(
        {0.0,
         0.0,
         0.0,
         0.0,
         2.50376224635538,
         -2.39920164674269,
         3.06882035390508,
         0.251214937225332,
         1.30010048888839,
         3.13839840845743});
    Eigen::Quaterniond expectedOut(1.0, 0.0, 0.0, 0.0);
    Eigen::Quaterniond out40 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 0, jointPositions2);
    expectedOut = Eigen::Quaterniond(0.46596194396055579, 0.0, 0.88480476195627467, 0.0);
    EXPECT_TRUE(quaterniondSubstractToVector(out40, expectedOut).norm() < PREC);
    Eigen::Quaterniond out41 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 2, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        -0.75467117770668013, 0.027507820221418688, -0.6550925574905061, -0.023842702993290183);
    EXPECT_TRUE(quaterniondSubstractToVector(out41, expectedOut).norm() < PREC);
    Eigen::Quaterniond out42 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 1, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        0.000036092953645694503,
        0.7551723414769772,
        -0.0000041151821363215418,
        -0.65552630256042632);
    EXPECT_TRUE(quaterniondSubstractToVector(out42, expectedOut).norm() < PREC);
    Eigen::Quaterniond out43 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 3, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        0.53636475699372121, -0.38770711938722691, -0.6479298647434778, 0.3770715149743129);
    EXPECT_TRUE(quaterniondSubstractToVector(out43, expectedOut).norm() < PREC);
    Eigen::Quaterniond out44 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 5, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        -0.53615551537512895, -0.83955309995055816, 0.080426545799125795, 0.03492028675511314);
    EXPECT_TRUE(quaterniondSubstractToVector(out44, expectedOut).norm() < PREC);
    Eigen::Quaterniond out45 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 4, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        0.080451685783086896, 0.034811219275352065, 0.53481269870386439, 0.84041125799950611);
    EXPECT_TRUE(quaterniondSubstractToVector(out45, expectedOut).norm() < PREC);
    Eigen::Quaterniond out46 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIE, 0, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        -0.53615551537512895, -0.83955309995055816, 0.080426545799125795, 0.03492028675511314);
    EXPECT_TRUE(quaterniondSubstractToVector(out46, expectedOut).norm() < PREC);
}

TEST_F(URDFKinematicChainShould, returnCorrectRotationPositionChange) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";

    std::string SPSRobotEndEffectorPoint("leafLink");
    sut_.reset(new crf::math::kinematicchain::URDFKinematicChain(
        pathToURDF, SPSRobotEndEffectorPoint, ""));
    crf::utility::types::JointPositions jointPositions0(
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions jointPositions1(
        {-0.029005763708726,
         0.182452167505983,
         -1.565056014150725,
         -0.084539479817724,
         -3.029177341404146,
         -0.457014640871583,
         1.242448406390738,
         -94.155944487570736,
         0.933728162671238,
         0.350321001356112});
    crf::utility::types::JointPositions jointPositions2(
        {0.0,
         0.0,
         0.0,
         0.0,
         2.50376224635538,
         -2.39920164674269,
         3.06882035390508,
         0.251214937225332,
         1.30010048888839,
         3.13839840845743});
    crf::utility::types::JointPositions jointPositions3(
        {0.0,
         5.0,
         4.0,
         -2.0,
         -0.206916683399114,
         0.931158050055648,
         -2.98307931430608,
         2.150147559173,
         0.370912419318411,
         2.22487559854808});
    crf::utility::types::JointPositions jointPositions4(
        {0.0,
         0.0,
         0.0,
         0.0,
         -2.30007778521014,
         -2.9475078947703,
         2.75920871534614,
         -1.24843081561655,
         -1.28469880703987,
         -1.0496922993298});
    crf::utility::types::JointPositions jointPositions5(
        {-0.7581, 2.0783, -2.2220, 0.4488, 0.0446, -0.7562, 0.4243, -0.7939, 0.8598, 8.5562});

    Eigen::Quaterniond expectedOut(1.0, 0.0, 0.0, 0.0);

    Eigen::Quaterniond out50 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 0, jointPositions0);
    expectedOut = Eigen::Quaterniond(0.98628560153796774, 0, -0.16504760585627751, 0.0);
    EXPECT_TRUE(quaterniondSubstractToVector(out50, expectedOut).norm() < PREC);
    Eigen::Quaterniond out51 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 2, jointPositions3);
    expectedOut = Eigen::Quaterniond(
        -0.66836361734732319, -0.053119212286656288, -0.73960597300901965, 0.058748863664137625);
    EXPECT_TRUE(quaterniondSubstractToVector(out51, expectedOut).norm() < PREC);
    Eigen::Quaterniond out52 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 1, jointPositions4);
    expectedOut = Eigen::Quaterniond(
        -3.4099499849878617e-05,
        0.15725334610732278,
        -0.000012524381696254691,
        0.98755829388366467);
    EXPECT_TRUE(quaterniondSubstractToVector(out52, expectedOut).norm() < PREC);
    Eigen::Quaterniond out53 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 3, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        0.53636475699372121, -0.38770711938722691, -0.6479298647434778, 0.3770715149743129);
    EXPECT_TRUE(quaterniondSubstractToVector(out53, expectedOut).norm() < PREC);
    Eigen::Quaterniond out54 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 5, jointPositions5);
    expectedOut = Eigen::Quaterniond(
        0.14644233151675626, -0.67706947050832189, -0.68106711295753775, -0.23723229816616925);
    EXPECT_TRUE(quaterniondSubstractToVector(out54, expectedOut).norm() < PREC);
    Eigen::Quaterniond out55 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 4, jointPositions1);
    expectedOut = Eigen::Quaterniond(
        +0.11626358254859705, 0.58008725722990584, -0.74396916777598643, -0.31063069837212998);
    EXPECT_TRUE(quaterniondSubstractToVector(out55, expectedOut).norm() < PREC);
    Eigen::Quaterniond out56 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIE, 0, jointPositions0);
    expectedOut = Eigen::Quaterniond(
        0.53296276972012624, -0.13883757523469051, -0.089125394437540126, 0.82989847443019005);
    EXPECT_TRUE(quaterniondSubstractToVector(out56, expectedOut).norm() < PREC);

    std::srand(56461384);
    std::vector<double> jointPositionssRandom;

    jointPositionssRandom.push_back(-0.029005763708726);
    jointPositionssRandom.push_back(0.182452167505983);
    jointPositionssRandom.push_back(-1.565056014150725);
    jointPositionssRandom.push_back(-0.084539479817724);
    jointPositionssRandom.push_back(-3.029177341404146);
    jointPositionssRandom.push_back(-0.457014640871583);
    jointPositionssRandom.push_back(1.242448406390738);
    jointPositionssRandom.push_back(-94.155944487570736);
    jointPositionssRandom.push_back(0.933728162671238);
    jointPositionssRandom.push_back(0.350321001356112);

    for (int i = 0; i < 100; i++) {
        jointPositionssRandom = jointPositionssRandom;
        double random_variable = static_cast<double>(std::rand()) / RAND_MAX;
        for (int k = 0; k < 10; k++) {
            if (static_cast<double>(std::rand()) / RAND_MAX < random_variable) {
                jointPositionssRandom[k] += static_cast<double>(std::rand()) * 10 / RAND_MAX - 5;
            }
        }
        crf::utility::types::JointPositions jointPositionsIntermediate(jointPositionssRandom);
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE, 0, jointPositionsIntermediate);
    }
    out50 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 0, jointPositions0);
    expectedOut = Eigen::Quaterniond(0.98628560153796774, 0, -0.16504760585627751, 0.0);
    EXPECT_TRUE(quaterniondSubstractToVector(out50, expectedOut).norm() < PREC);
    out51 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 2, jointPositions3);
    expectedOut = Eigen::Quaterniond(
        -0.66836361734732319, -0.053119212286656288, -0.73960597300901965, 0.058748863664137625);
    EXPECT_TRUE(quaterniondSubstractToVector(out51, expectedOut).norm() < PREC);
    out52 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 1, jointPositions4);
    expectedOut = Eigen::Quaterniond(
        -3.4099499849878617e-05,
        0.15725334610732278,
        -0.000012524381696254691,
        0.98755829388366467);
    EXPECT_TRUE(quaterniondSubstractToVector(out52, expectedOut).norm() < PREC);
    out53 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 3, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        0.53636475699372121, -0.38770711938722691, -0.6479298647434778, 0.3770715149743129);
    EXPECT_TRUE(quaterniondSubstractToVector(out53, expectedOut).norm() < PREC);
    out54 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 5, jointPositions5);
    expectedOut = Eigen::Quaterniond(
        0.14644233151675626, -0.67706947050832189, -0.68106711295753775, -0.23723229816616925);
    EXPECT_TRUE(quaterniondSubstractToVector(out54, expectedOut).norm() < PREC);
    out55 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 4, jointPositions1);
    expectedOut = Eigen::Quaterniond(
        +0.11626358254859705, 0.58008725722990584, -0.74396916777598643, -0.31063069837212998);
    EXPECT_TRUE(quaterniondSubstractToVector(out55, expectedOut).norm() < PREC);
    out56 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIE, 0, jointPositions0);
    expectedOut = Eigen::Quaterniond(
        0.53296276972012624, -0.13883757523469051, -0.089125394437540126, 0.82989847443019005);
    EXPECT_TRUE(quaterniondSubstractToVector(out56, expectedOut).norm() < PREC);
}

TEST_F(URDFKinematicChainShould, returnCorrectRotationPositionChangeFixedJointsAndTool) {
    std::string pathToURDF = pathToURDFs_ + "/KinematicChainFixedJointsTest.urdf";

    std::string endEffectorName("leafLink");

    std::string toolPath = pathToURDFs_ + "/KinematicChainToolTest.urdf";

    sut_.reset(
        new crf::math::kinematicchain::URDFKinematicChain(pathToURDF, endEffectorName, toolPath));
    crf::utility::types::JointPositions jointPositions0({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions jointPositions1(
        {-3.029177341404146,
         -0.457014640871583,
         1.242448406390738,
         -94.155944487570736,
         0.933728162671238,
         0.350321001356112});
    crf::utility::types::JointPositions jointPositions2(
        {2.50376224635538,
         -2.39920164674269,
         3.06882035390508,
         0.251214937225332,
         1.30010048888839,
         3.13839840845743});
    crf::utility::types::JointPositions jointPositions3(
        {-0.206916683399114,
         0.931158050055648,
         -2.98307931430608,
         2.150147559173,
         0.370912419318411,
         2.22487559854808});
    crf::utility::types::JointPositions jointPositions4(
        {-2.30007778521014,
         -2.9475078947703,
         2.75920871534614,
         -1.24843081561655,
         -1.28469880703987,
         -1.0496922993298});
    crf::utility::types::JointPositions jointPositions5(
        {0.0446, -0.7562, 0.4243, -0.7939, 0.8598, 8.5562});

    Eigen::Quaterniond expectedOut(1.0, 0.0, 0.0, 0.0);

    Eigen::Quaterniond out50 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 0, jointPositions0);
    expectedOut = Eigen::Quaterniond(-3.673205103346574e-06, -0.9999999999932538, 0.0, 0.0);
    EXPECT_TRUE(quaterniondSubstractToVector(out50, expectedOut).norm() < PREC);
    Eigen::Quaterniond out51 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 2, jointPositions3);
    expectedOut = Eigen::Quaterniond(
        -0.5636407351016455, 0.6393471817193719, -0.3020617894470382, -0.4269695285586864);
    EXPECT_TRUE(quaterniondSubstractToVector(out51, expectedOut).norm() < PREC);
    Eigen::Quaterniond out52 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 1, jointPositions4);
    expectedOut = Eigen::Quaterniond(
        0.6703833115313872, 0.6144087796087391, -0.3499965247764764, -0.2249233198321166);
    EXPECT_TRUE(quaterniondSubstractToVector(out52, expectedOut).norm() < PREC);
    Eigen::Quaterniond out53 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 3, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        0.1412912026531128, -0.9268834081742769, -0.1814386348803129, -0.2966546232183863);
    EXPECT_TRUE(quaterniondSubstractToVector(out53, expectedOut).norm() < PREC);
    Eigen::Quaterniond out54 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 5, jointPositions5);
    expectedOut = Eigen::Quaterniond(
        -0.3116211961719286, -0.6497107412077507, 0.6876826268992036, -0.08866108231159747);
    EXPECT_TRUE(quaterniondSubstractToVector(out54, expectedOut).norm() < PREC);
    Eigen::Quaterniond out55 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 4, jointPositions1);
    expectedOut = Eigen::Quaterniond(
        -0.4909997691088272, -0.4260723309249287, 0.5923574987406133, -0.4759140565701354);
    EXPECT_TRUE(quaterniondSubstractToVector(out55, expectedOut).norm() < PREC);
    Eigen::Quaterniond out56 =
        sut_->computeRotation(crf::math::kinematicchain::Rotations::IIE, 0, jointPositions0);
    expectedOut = Eigen::Quaterniond(
        -0.2300444277233708, 0.7535267363717745, -0.3367280415423212, -0.5156464339888355);
    EXPECT_TRUE(quaterniondSubstractToVector(out56, expectedOut).norm() < PREC);

    std::srand(56461384);
    std::vector<double> jointPositionssRandom;

    jointPositionssRandom.push_back(-3.029177341404146);
    jointPositionssRandom.push_back(-0.457014640871583);
    jointPositionssRandom.push_back(1.242448406390738);
    jointPositionssRandom.push_back(-94.155944487570736);
    jointPositionssRandom.push_back(0.933728162671238);
    jointPositionssRandom.push_back(0.350321001356112);

    int chainSize = sut_->getChainSize();

    for (int i = 0; i < 100; i++) {
        jointPositionssRandom = jointPositionssRandom;
        double random_variable = static_cast<double>(std::rand()) / RAND_MAX;
        for (int k = 0; k < chainSize; k++) {
            if (static_cast<double>(std::rand()) / RAND_MAX < random_variable) {
                jointPositionssRandom[k] += static_cast<double>(std::rand()) * 10 / RAND_MAX - 5;
            }
        }
        crf::utility::types::JointPositions jointPositionsIntermediate(jointPositionssRandom);
        sut_->computeTranslation(
            crf::math::kinematicchain::Translations::IIE, 0, jointPositionsIntermediate);
    }
    out50 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 0, jointPositions0);
    expectedOut = Eigen::Quaterniond(-3.673205103346574e-06, -0.9999999999932538, 0.0, 0.0);
    EXPECT_TRUE(quaterniondSubstractToVector(out50, expectedOut).norm() < PREC);
    out51 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 2, jointPositions3);
    expectedOut = Eigen::Quaterniond(
        -0.5636407351016455, 0.6393471817193719, -0.3020617894470382, -0.4269695285586864);
    EXPECT_TRUE(quaterniondSubstractToVector(out51, expectedOut).norm() < PREC);
    out52 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 1, jointPositions4);
    expectedOut = Eigen::Quaterniond(
        0.6703833115313872, 0.6144087796087391, -0.3499965247764764, -0.2249233198321166);
    EXPECT_TRUE(quaterniondSubstractToVector(out52, expectedOut).norm() < PREC);
    out53 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 3, jointPositions2);
    expectedOut = Eigen::Quaterniond(
        0.1412912026531128, -0.9268834081742769, -0.1814386348803129, -0.2966546232183863);
    EXPECT_TRUE(quaterniondSubstractToVector(out53, expectedOut).norm() < PREC);
    out54 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 5, jointPositions5);
    expectedOut = Eigen::Quaterniond(
        -0.3116211961719286, -0.6497107412077507, 0.6876826268992036, -0.08866108231159747);
    EXPECT_TRUE(quaterniondSubstractToVector(out54, expectedOut).norm() < PREC);
    out55 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIJ, 4, jointPositions1);
    expectedOut = Eigen::Quaterniond(
        -0.4909997691088272, -0.4260723309249287, 0.5923574987406133, -0.4759140565701354);
    EXPECT_TRUE(quaterniondSubstractToVector(out55, expectedOut).norm() < PREC);
    out56 = sut_->computeRotation(crf::math::kinematicchain::Rotations::IIE, 0, jointPositions0);
    expectedOut = Eigen::Quaterniond(
        -0.2300444277233708, 0.7535267363717745, -0.3367280415423212, -0.5156464339888355);
    EXPECT_TRUE(quaterniondSubstractToVector(out56, expectedOut).norm() < PREC);
}

Eigen::Vector4d quaterniondSubstractToVector(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
    return Eigen::Vector4d(q1.w() - q2.w(), q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z());
}
