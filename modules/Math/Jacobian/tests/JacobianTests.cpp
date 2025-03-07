/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *         Ante Marić CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include "Jacobian/KinChainJacobian/KinChainJacobian.hpp"
#include "Jacobian/MathExprJacobian/MathExprJacobian.hpp"
#include "KinematicChain/URDFKinematicChain/URDFKinematicChain.hpp"

using crf::math::jacobian::IJacobian;
using crf::math::jacobian::KinChainJacobian;
using crf::math::jacobian::MathExprJacobian;
using crf::math::kinematicchain::IKinematicChain;
using crf::math::kinematicchain::URDFKinematicChain;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskSpaceTangentDimension;

Eigen::MatrixXd eigenParser(const JointPositions& qValues) {
    double q1 = qValues[0];  // changed by Jorge
    double q2 = qValues[1];
    double q3 = qValues[2];
    double q4 = qValues[3];
    double q5 = qValues[4];

    double l3x = 0.6127;
    double l4x = 0.57155;

    double l6y = 0.11655;

    double l5z = 0.17415;
    double l6z = 0.11985;

    Eigen::MatrixXd j(6, 6);
    j(0, 0) = (((cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x) * cos(q3) +
                (-sin(q4) * sin(q5) * l6y - l6z * cos(q4)) * sin(q3) + l3x) *
                   cos(q2) -
               ((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) -
                sin(q3) * (-cos(q4) * sin(q5) * l6y + l6z * sin(q4) - l4x)) *
                   sin(q2)) *
            sin(q1) +
        cos(q1) * (cos(q5) * l6y + l5z);
    j(0, 1) = (((cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x) * cos(q3) +
                (-sin(q4) * sin(q5) * l6y - l6z * cos(q4)) * sin(q3) + l3x) *
                   sin(q2) +
               ((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) +
                sin(q3) * (cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x)) *
                   cos(q2)) *
        cos(q1);
    j(0, 2) = (((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) +
                sin(q3) * (cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x)) *
                   cos(q2) +
               ((cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x) * cos(q3) -
                (sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * sin(q3)) *
                   sin(q2)) *
        cos(q1);
    j(0, 3) = cos(q1) *
        (((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) +
          (cos(q4) * sin(q5) * l6y - l6z * sin(q4)) * sin(q3)) *
             cos(q2) +
         ((cos(q4) * sin(q5) * l6y - l6z * sin(q4)) * cos(q3) -
          (sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * sin(q3)) *
             sin(q2));
    j(0, 4) = l6y *
        (((sin(q3) * sin(q4) - cos(q3) * cos(q4)) * cos(q2) +
          sin(q2) * (sin(q3) * cos(q4) + sin(q4) * cos(q3))) *
             cos(q5) * cos(q1) -
         sin(q1) * sin(q5));
    j(0, 5) = 0;
    j(1, 0) = (((-cos(q4) * sin(q5) * l6y + l6z * sin(q4) - l4x) * cos(q3) +
                (sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * sin(q3) - l3x) *
                   cos(q2) +
               ((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) -
                sin(q3) * (-cos(q4) * sin(q5) * l6y + l6z * sin(q4) - l4x)) *
                   sin(q2)) *
            cos(q1) +
        sin(q1) * (cos(q5) * l6y + l5z);
    j(1, 1) = sin(q1) *
        (((cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x) * cos(q3) +
          (-sin(q4) * sin(q5) * l6y - l6z * cos(q4)) * sin(q3) + l3x) *
             sin(q2) +
         ((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) +
          sin(q3) * (cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x)) *
             cos(q2));
    j(1, 2) = (((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) +
                sin(q3) * (cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x)) *
                   cos(q2) +
               ((cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x) * cos(q3) -
                (sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * sin(q3)) *
                   sin(q2)) *
        sin(q1);
    j(1, 3) = -(((-sin(q4) * sin(q5) * l6y - l6z * cos(q4)) * cos(q3) -
                 (cos(q4) * sin(q5) * l6y - l6z * sin(q4)) * sin(q3)) *
                    cos(q2) +
                ((-cos(q4) * sin(q5) * l6y + l6z * sin(q4)) * cos(q3) +
                 (sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * sin(q3)) *
                    sin(q2)) *
        sin(q1);
    j(1, 4) = (((sin(q3) * sin(q4) - cos(q3) * cos(q4)) * cos(q2) +
                sin(q2) * (sin(q3) * cos(q4) + sin(q4) * cos(q3))) *
                   sin(q1) * cos(q5) +
               cos(q1) * sin(q5)) *
        l6y;
    j(1, 5) = 0;
    j(2, 0) = 0;
    j(2, 1) = ((-cos(q4) * sin(q5) * l6y + l6z * sin(q4) - l4x) * cos(q3) +
               (sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * sin(q3) - l3x) *
            cos(q2) +
        sin(q2) *
            ((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) +
             sin(q3) * (cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x));
    j(2, 2) = ((-cos(q4) * sin(q5) * l6y + l6z * sin(q4) - l4x) * cos(q3) +
               (sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * sin(q3)) *
            cos(q2) +
        sin(q2) *
            ((sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * cos(q3) +
             sin(q3) * (cos(q4) * sin(q5) * l6y - l6z * sin(q4) + l4x));
    j(2, 3) = ((-cos(q4) * sin(q5) * l6y + l6z * sin(q4)) * cos(q3) +
               (sin(q4) * sin(q5) * l6y + l6z * cos(q4)) * sin(q3)) *
            cos(q2) -
        sin(q2) *
            ((-sin(q4) * sin(q5) * l6y - l6z * cos(q4)) * cos(q3) +
             sin(q3) * (-cos(q4) * sin(q5) * l6y + l6z * sin(q4)));
    j(2, 4) = l6y *
        (sin(q4) * (sin(q2) * sin(q3) - cos(q2) * cos(q3)) -
         (sin(q2) * cos(q3) + cos(q2) * sin(q3)) * cos(q4)) *
        cos(q5);
    j(2, 5) = 0;
    j(3, 0) = 0;
    j(3, 1) = sin(q1);
    j(3, 2) = sin(q1);
    j(3, 3) = sin(q1);
    j(3, 4) = -cos(q1) *
        (sin(q4) * (sin(q2) * sin(q3) - cos(q2) * cos(q3)) -
         (sin(q2) * cos(q3) + cos(q2) * sin(q3)) * cos(q4));
    j(3, 5) = sin(q5) *
            ((sin(q3) * sin(q4) - cos(q3) * cos(q4)) * cos(q2) +
             sin(q2) * (sin(q3) * cos(q4) + sin(q4) * cos(q3))) *
            cos(q1) +
        sin(q1) * cos(q5);
    j(4, 0) = 0;
    j(4, 1) = -cos(q1);
    j(4, 2) = -cos(q1);
    j(4, 3) = -cos(q1);
    j(4, 4) = -sin(q1) *
        (sin(q4) * (sin(q2) * sin(q3) - cos(q2) * cos(q3)) -
         (sin(q2) * cos(q3) + cos(q2) * sin(q3)) * cos(q4));
    j(4, 5) = sin(q5) *
            ((sin(q3) * sin(q4) - cos(q3) * cos(q4)) * cos(q2) +
             sin(q2) * (sin(q3) * cos(q4) + sin(q4) * cos(q3))) *
            sin(q1) -
        cos(q1) * cos(q5);
    j(5, 0) = 1;
    j(5, 1) = 0;
    j(5, 2) = 0;
    j(5, 3) = 0;
    j(5, 4) = (sin(q2) * cos(q3) + cos(q2) * sin(q3)) * sin(q4) -
        (-sin(q2) * sin(q3) + cos(q2) * cos(q3)) * cos(q4);
    j(5, 5) = -((sin(q2) * cos(q3) + cos(q2) * sin(q3)) * cos(q4) +
                (-sin(q2) * sin(q3) + cos(q2) * cos(q3)) * sin(q4)) *
        sin(q5);

    return j;
}

class JacobianShould : public ::testing::Test {
 protected:
    JacobianShould() :
        logger_("JacobianShould"),
        maxAbsErr_(1e-6) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~JacobianShould() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    void SetUp() override {
        std::string jsonPath = __FILE__;
        jsonPath = jsonPath.substr(0, jsonPath.find("Jacobian/"));
        jsonPath += "Jacobian/tests/config/Jacobian.json";
        std::ifstream jsonConfigPath(jsonPath);
        jsonConfig_ = nlohmann::json::parse(jsonConfigPath);
        lx_ = {0.0, 0.0, 0.6127, 0.57155, 0.0, 0.0};
        ly_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.11655};
        lz_ = {0.0, 0.1807, 0.0, 0.0, 0.17415, 0.11985};

        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("Math/"));
        testDirName_ += "Math/Jacobian/tests/config";

        std::string pathToURDF = testDirName_ + "/KinematicChainCombinedTestingBasedOnSPS.urdf";
        std::string toolName = "kinova_end_effector";

        kinChain_.reset(new URDFKinematicChain(pathToURDF, toolName, ""));
        taskSpace_ = TaskSpace();

        pathToURDF = testDirName_ + "/KinematicChainArmTesting.urdf";
        toolName = "ee_link";

        kinChainArm_.reset(new URDFKinematicChain(pathToURDF, toolName, ""));
        taskSpaceArm_ = TaskSpace();

        pathToURDF = testDirName_ + "/KinematicChainPlatformTestingBasedOnSPS.urdf";
        toolName = "platform_root_link";

        kinChainPlatform_.reset(new URDFKinematicChain(pathToURDF, toolName, ""));
        taskSpacePlatform_ = TaskSpace();
        taskSpacePlatform_[TaskSpaceTangentDimension::Vx] = true;
        taskSpacePlatform_[TaskSpaceTangentDimension::Vy] = true;
        taskSpacePlatform_[TaskSpaceTangentDimension::Vz] = false;
        taskSpacePlatform_[TaskSpaceTangentDimension::Wx] = false;
        taskSpacePlatform_[TaskSpaceTangentDimension::Wy] = false;
        taskSpacePlatform_[TaskSpaceTangentDimension::Wz] = true;

        pathToURDF = testDirName_ + "/KinematicChainPlatformTestingBasedOnSPSWrongTaskSpace.urdf";
        toolName = "platform_root_link";

        kinChainPlatformWrongTaskSpace_.reset(new URDFKinematicChain(pathToURDF, toolName, ""));
        taskSpacePlatformWrong_ = TaskSpace();
        taskSpacePlatformWrong_[TaskSpaceTangentDimension::Vx] = true;
        taskSpacePlatformWrong_[TaskSpaceTangentDimension::Vy] = true;
        taskSpacePlatformWrong_[TaskSpaceTangentDimension::Vz] = false;
        taskSpacePlatformWrong_[TaskSpaceTangentDimension::Wx] = true;
        taskSpacePlatformWrong_[TaskSpaceTangentDimension::Wy] = false;
        taskSpacePlatformWrong_[TaskSpaceTangentDimension::Wz] = true;
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<IJacobian> sut_;
    nlohmann::json jsonConfig_;
    std::vector<double> lx_, ly_, lz_;
    std::shared_ptr<IKinematicChain> kinChain_;
    std::shared_ptr<IKinematicChain> kinChainArm_;
    std::shared_ptr<IKinematicChain> kinChainPlatform_;
    std::shared_ptr<IKinematicChain> kinChainPlatformWrongTaskSpace_;
    TaskSpace taskSpace_;
    TaskSpace taskSpaceArm_;
    TaskSpace taskSpacePlatform_;
    TaskSpace taskSpacePlatformWrong_;
    std::string testDirName_;
    double maxAbsErr_;
};

TEST_F(JacobianShould, comparisonBetweenEigenAndCurrentParser) {
    clock_t realTime;
    double t = 0.0;

    JointPositions q(
        {5 * M_PI / 180,
         5 * M_PI / 180,
         5 * M_PI / 180,
         5 * M_PI / 180,
         5 * M_PI / 180,
         5 * M_PI / 180});

    realTime = clock();
    Eigen::MatrixXd JMatrixEigen = eigenParser(q);
    realTime = clock() - realTime;
    t = static_cast<double>(realTime) / CLOCKS_PER_SEC;
    logger_->info("JMatrixEigen tooks {} milliseconds", t * 1000);

    ASSERT_NO_THROW(sut_.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz_)));
    realTime = clock();
    Eigen::MatrixXd JMatrixCP = sut_->evaluate(q);
    realTime = clock() - realTime;
    t = static_cast<double>(realTime) / CLOCKS_PER_SEC;
    logger_->info("JMatrixCP tooks {} milliseconds", t * 1000);

    for (unsigned int row = 0; row < 6; row++) {
        for (unsigned int column = 0; column < 6; column++) {
            ASSERT_DOUBLE_EQ(JMatrixEigen(row, column), JMatrixCP(row, column));
        }
    }
}

TEST_F(JacobianShould, ThrowVariablesAreNotCorrectlyInitialized) {
    std::vector<double> lz = {0.0, 0.1807, 0.0, 0.0, 0.17415};

    ASSERT_THROW(
        sut_.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz)), std::invalid_argument);
}

TEST_F(JacobianShould, ThrowMathExpressionEmpty) {
    nlohmann::json jsonConfig;

    ASSERT_THROW(
        sut_.reset(new MathExprJacobian(jsonConfig, lx_, ly_, lz_)), std::invalid_argument);
}

TEST_F(JacobianShould, ThrowMathExpressionWrongFormat) {
    int numberOfTests = 7;
    logger_->info("There are {} format tests", numberOfTests);
    for (int i = 0; i < numberOfTests; i++) {
        logger_->info("Test number {}", i);
        std::string jsonPath = __FILE__;
        jsonPath = jsonPath.substr(0, jsonPath.find("Jacobian/"));
        jsonPath += "Jacobian/tests/config/JacobianWrongFormat" + std::to_string(i) + ".json";
        std::ifstream jsonConfigPath(jsonPath);
        nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);

        ASSERT_THROW(
            sut_.reset(new MathExprJacobian(jsonConfig, lx_, ly_, lz_)), std::invalid_argument);
    }
}

TEST_F(JacobianShould, throwErrorJacobianWithMoreRowsThanColumsTest) {
    std::string jsonPath = __FILE__;
    jsonPath = jsonPath.substr(0, jsonPath.find("Jacobian/"));
    jsonPath += "Jacobian/tests/config/JacobianWrongSize.json";
    std::ifstream jsonConfigPath(jsonPath);
    nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);

    std::vector<double> lx = {0.0, 0.6127};
    std::vector<double> ly = {0.0, 0.11655};
    std::vector<double> lz = {0.1807, 0.0};

    ASSERT_THROW(sut_.reset(new MathExprJacobian(jsonConfig, lx, ly, lz)), std::invalid_argument);
}

// TEST_F(JacobianShould, returnEmptyMatrixIfInputDimensionQIsDifferentThanTheExpected)
// {
//     ASSERT_NO_THROW(sut_.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz_)));

//     JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0});
//     Eigen::MatrixXd JMatrix = sut_->evaluate(q);
//     Eigen::MatrixXd nullMatrix;
//     ASSERT_EQ(JMatrix, nullMatrix);
// }

// TEST_F(JacobianShould, returnFalseManipulabilityIfInputDimensionQIsDifferentThanTheExpected)
// {
//     ASSERT_NO_THROW(sut_.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz_)));

//     JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0});
//     std::optional<double> w = sut_->getKinematicManipulability(q);
//     ASSERT_FALSE(w);
//     ASSERT_EQ(w, std::nullopt);
// }

TEST_F(JacobianShould, returnCorrectResultsMathExpr) {
    ASSERT_NO_THROW(sut_.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz_)));

    JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Eigen::MatrixXd JMatrix = sut_->evaluate(q);
    ASSERT_DOUBLE_EQ(JMatrix(0, 0), 0.2907);
    ASSERT_DOUBLE_EQ(JMatrix(0, 1), 0.11985);
    ASSERT_DOUBLE_EQ(JMatrix(0, 2), 0.11985);
    ASSERT_DOUBLE_EQ(JMatrix(0, 3), 0.11985);
    ASSERT_DOUBLE_EQ(JMatrix(0, 4), -0.11655);
    ASSERT_DOUBLE_EQ(JMatrix(0, 5), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(1, 0), -1.18425);
    ASSERT_DOUBLE_EQ(JMatrix(1, 1), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(1, 2), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(1, 3), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(1, 4), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(1, 5), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(2, 0), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(2, 1), -1.18425);
    ASSERT_DOUBLE_EQ(JMatrix(2, 2), -0.57155);
    ASSERT_DOUBLE_EQ(JMatrix(2, 3), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(2, 4), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(2, 5), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(3, 0), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(3, 1), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(3, 2), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(3, 3), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(3, 4), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(3, 5), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(4, 0), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(4, 1), -1.0);
    ASSERT_DOUBLE_EQ(JMatrix(4, 2), -1.0);
    ASSERT_DOUBLE_EQ(JMatrix(4, 3), -1.0);
    ASSERT_DOUBLE_EQ(JMatrix(4, 4), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(4, 5), -1.0);
    ASSERT_DOUBLE_EQ(JMatrix(5, 0), 1.0);
    ASSERT_DOUBLE_EQ(JMatrix(5, 1), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(5, 2), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(5, 3), 0.0);
    ASSERT_DOUBLE_EQ(JMatrix(5, 4), -1.0);
    ASSERT_DOUBLE_EQ(JMatrix(5, 5), 0.0);

    std::optional<double> w = sut_->getKinematicManipulability(q);
    ASSERT_TRUE(w);
    ASSERT_EQ(w.value(), 0);
}

// TEST_F(JacobianShould, returnCorrectResultsKinChainArmOnly)
// {
//     std::string pathToURDF2 = testDirName_ + "/ur10e.urdf";
//     std::string tool2 = "tool0";
//     std::shared_ptr<IKinematicChain> kinChain2 =
//     std::make_shared<URDFKinematicChain>(pathToURDF2, tool2);

//     JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//     ASSERT_NO_THROW(sut_.reset(new KinChainJacobian(kinChain2)));
//     Eigen::MatrixXd JMatrix = sut_->evaluate(q);
//     ASSERT_NEAR(JMatrix(0, 0), -1.176, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 1), 1.304, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 2), 0.691, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 3), 0.12, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 4), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 5), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 0), 0.002, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 1), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 2), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 3), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 4), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 5), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 0), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 1), -0.002, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 2), 0.135, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 3), 0.135, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 4), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 5), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 0), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 1), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 2), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 3), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 4), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 5), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 0), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 1), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 2), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 3), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 4), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 5), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 0), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 1), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 2), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 3), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 4), -1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 5), 0.0, maxAbsErr_);
// }

// TEST_F(JacobianShould, returnCorrectResultsKinChainCombined)
// {
//     JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//     ASSERT_NO_THROW(sut_.reset(new KinChainJacobian(kinChain_)));
//     Eigen::MatrixXd JMatrix = sut_->evaluate(q);
//     ASSERT_NEAR(JMatrix(0, 0), 0.05, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 1), 0.05, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 2), 0.05, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 3), 0.05, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 4), -1.176, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 5), 1.304, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 6), 0.691, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 7), 0.12, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 8), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(0, 9), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 0), 0.05, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 1), -0.05, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 2), 0.05, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 3), -0.05, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 4), 0.202, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 5), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 6), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 7), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 8), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(1, 9), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 0), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 1), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 2), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 3), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 4), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 5), -0.002, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 6), 0.135, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 7), 0.135, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 8), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(2, 9), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 0), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 1), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 2), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 3), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 4), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 5), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 6), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 7), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 8), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(3, 9), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 0), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 1), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 2), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 3), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 4), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 5), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 6), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 7), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 8), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(4, 9), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 0), -0.0405, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 1), 0.0405, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 2), 0.0405, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 3), -0.0405, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 4), 1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 5), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 6), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 7), 0.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 8), -1.0, maxAbsErr_);
//     ASSERT_NEAR(JMatrix(5, 9), 0.0, maxAbsErr_);
// }
/*
TEST_F(JacobianShould, EqualityDisequalityTestMathExpr) {
    std::unique_ptr<IJacobian> sut1;
    std::unique_ptr<IJacobian> sut2;
    std::unique_ptr<IJacobian> sut3;

    ASSERT_NO_THROW(sut1.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz_)));
    ASSERT_NO_THROW(sut2.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz_)));

    std::vector<double> lz = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ASSERT_NO_THROW(sut3.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz)));

    ASSERT_TRUE(*sut1.get() == *sut1.get());
    ASSERT_TRUE(*sut1.get() == *sut2.get());
    ASSERT_FALSE(*sut1.get() == *sut3.get());
    ASSERT_FALSE(*sut1.get() != *sut1.get());
    ASSERT_FALSE(*sut1.get() != *sut2.get());
    ASSERT_TRUE(*sut1.get() != *sut3.get());
}

TEST_F(JacobianShould, EqualityDisequalityTestKinChain) {
    std::unique_ptr<IJacobian> sut1;
    std::unique_ptr<IJacobian> sut2;
    std::unique_ptr<IJacobian> sut3;

    std::string pathToURDF2 = testDirName_ + "/j2s6s200.urdf";
    std::string tool2 = "j2s6s200_end_effector";
    std::shared_ptr<IKinematicChain> kinChain2 = std::make_shared<URDFKinematicChain>(pathToURDF2,
        tool2);

    ASSERT_NO_THROW(sut1.reset(new KinChainJacobian(kinChain_)));
    ASSERT_NO_THROW(sut2.reset(new KinChainJacobian(kinChain_)));
    ASSERT_NO_THROW(sut3.reset(new KinChainJacobian(kinChain2)));

    ASSERT_TRUE(*sut1.get() == *sut1.get());
    ASSERT_TRUE(*sut1.get() == *sut2.get());
    ASSERT_FALSE(*sut1.get() == *sut3.get());
    ASSERT_FALSE(*sut1.get() != *sut1.get());
    ASSERT_FALSE(*sut1.get() != *sut2.get());
    ASSERT_TRUE(*sut1.get() != *sut3.get());
}

TEST_F(JacobianShould, CopyCTorAndOperatorEqualTestMathExpr) {
    MathExprJacobian J(jsonConfig_, lx_, ly_, lz_);
    JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Eigen::MatrixXd evalJ = J.evaluate(q);

    MathExprJacobian copyJ(J);
    JointPositions copyq(q);
    Eigen::MatrixXd evalJWithBothCopies = copyJ.evaluate(copyq);
    ASSERT_TRUE(evalJ == evalJWithBothCopies);

    // If the copyq changes, the evaluation now changes because the exprtk::expression<double> and
    //     the exprtk::symbol_table<double> is being constructed again in the CopyCTor.
    //     If both are just copied as the rest of variables, the result evalJWithBothCopies will be
    //     equal to evalJWithBothCopies2.
    copyq = JointPositions({0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    Eigen::MatrixXd evalJWithBothCopies2 = copyJ.evaluate(copyq);
    ASSERT_TRUE(evalJWithBothCopies != evalJWithBothCopies2);
    // ASSERT_TRUE(evalJWithBothCopies == evalJWithBothCopies2);

    std::vector<double> lz2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    MathExprJacobian equalJ(jsonConfig_, lx_, ly_, lz2);
    Eigen::MatrixXd evalJWithEqualJ = equalJ.evaluate(q);
    ASSERT_TRUE(evalJ != evalJWithEqualJ);

    JointPositions equalq = JointPositions({0.3, 0.3, 0.3, 0.3, 0.3, 0.3});
    Eigen::MatrixXd evalJWithBothEquals = equalJ.evaluate(equalq);
    ASSERT_TRUE(evalJWithEqualJ != evalJWithBothEquals);

    equalJ = J;
    equalq = q;
    Eigen::MatrixXd evalJWithBothEquals2 = equalJ.evaluate(equalq);
    ASSERT_TRUE(evalJ == evalJWithBothEquals2);

    // If the equalq changes, as same as in copyq case, the evaluation now changes because the
    //     exprtk::expression<double> and the exprtk::symbol_table<double> is being constructed
    //     again in the CopyCTor.
    //     If both are just equaled as the rest of variables, the result evalJWithBothEquals2 will
    //     be equal to evalJWithBothEquals3.
    equalq = JointPositions({0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    Eigen::MatrixXd evalJWithBothEquals3 = equalJ.evaluate(equalq);
    ASSERT_TRUE(evalJWithBothEquals2 != evalJWithBothEquals3);
    // ASSERT_TRUE(evalJWithBothEquals2 == evalJWithBothEquals3);


    // This situation is happening because in exprtk.hpp library, variable inputs in
    //     exprtk::symbol_table<double> are being taken as reference (&) so when you copy
    //     exprtk::symbol_table<double> and also copy the input variable, and then the variable
    //     copy is changed, the result in exprtk::symbol_table<double> doesn't change since it is
    //     refering to the original variable.
    //     Also the input in exprtk::expression<double>, that is exprtk::symbol_table<double>, is
    //     taken as a reference, so the same problem appeares again.
    //     There is the same problem with operator=.
}

TEST_F(JacobianShould, CopyCTorAndOperatorEqualTestKinChain) {
    KinChainJacobian J(kinChain_);
    JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Eigen::MatrixXd evalJ = J.evaluate(q);
    KinChainJacobian copyJ(J);
    JointPositions copyq(q);
    Eigen::MatrixXd evalJCopy = copyJ.evaluate(copyq);
    ASSERT_TRUE(evalJ == evalJCopy);
    ASSERT_TRUE(J == copyJ);
    ASSERT_FALSE(J != copyJ);

    JointPositions q2 = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::string pathToURDF2 = testDirName_ + "/j2s6s200.urdf";
    std::string tool2 = "j2s6s200_end_effector";
    std::shared_ptr<IKinematicChain> kinChain2 = std::make_shared<URDFKinematicChain>(pathToURDF2,
        tool2);

    KinChainJacobian J2(kinChain2);
    ASSERT_TRUE(J != J2);
    ASSERT_FALSE(J == J2);

    std::string pathToURDF3 = testDirName_ + "/ur10e.urdf";
    std::string tool3 = "tool0";
    std::shared_ptr<IKinematicChain> kinChain3 = std::make_shared<URDFKinematicChain>(pathToURDF3,
        tool3);
    KinChainJacobian J3(kinChain3);
    ASSERT_TRUE(J != J3);
    ASSERT_FALSE(J == J3);

    Eigen::MatrixXd evalJ2 = J2.evaluate(q2);
    Eigen::MatrixXd evalJ3 = J3.evaluate(q2);

    ASSERT_TRUE(evalJ2 != evalJ3);
    ASSERT_TRUE(J2 != J3);
    ASSERT_FALSE(J2 == J3);
}

TEST_F(JacobianShould, SeveralConstructorsTest) {
    JointPositions q1 = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    JointPositions q2 = JointPositions({0.0, 0.0, 0.3, 0.0, 0.0, 0.0});

    sut_.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz_));
    Eigen::MatrixXd JMatrix11 = sut_->evaluate(q1);
    Eigen::MatrixXd JMatrix12 = sut_->evaluate(q2);
    ASSERT_TRUE(JMatrix11 != JMatrix12);

    std::vector<double> lz2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    sut_.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz2));
    Eigen::MatrixXd JMatrix21 = sut_->evaluate(q1);
    Eigen::MatrixXd JMatrix22 = sut_->evaluate(q2);
    ASSERT_TRUE(JMatrix21 != JMatrix22);

    std::vector<double> lz3 = {0.0, 0.1807, 0.0, 0.0, 0.17415, 0.11985};
    sut_.reset(new MathExprJacobian(jsonConfig_, lx_, ly_, lz3));
    Eigen::MatrixXd JMatrix31 = sut_->evaluate(q1);
    Eigen::MatrixXd JMatrix32 = sut_->evaluate(q2);
    ASSERT_TRUE(JMatrix31 != JMatrix32);

    ASSERT_TRUE(JMatrix11 != JMatrix21);
    ASSERT_TRUE(JMatrix11 == JMatrix31);
    ASSERT_TRUE(JMatrix21 != JMatrix31);

    ASSERT_TRUE(JMatrix12 != JMatrix22);
    ASSERT_TRUE(JMatrix12 == JMatrix32);
    ASSERT_TRUE(JMatrix22 != JMatrix32);
}

TEST_F(JacobianShould, getJacobianRowsAndColsTest) {
    std::string jsonPath = __FILE__;
    jsonPath = jsonPath.substr(0, jsonPath.find("Jacobian/"));
    jsonPath += "Jacobian/tests/config/JacobianSmall.json";
    std::ifstream jsonConfigPath(jsonPath);
    nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);
    std::vector<double> lx = {0.0, 0.6127};
    std::vector<double> ly = {0.0, 0.11655};
    std::vector<double> lz = {0.1807, 0.0};

    MathExprJacobian j(jsonConfig, lx, ly, lz);
    ASSERT_EQ(j.rows(), 2);
    ASSERT_EQ(j.cols(), lx.size());

    KinChainJacobian j2(kinChain_);
    ASSERT_EQ(j2.rows(), 6);
    ASSERT_EQ(j2.cols(), 10);
}

TEST_F(JacobianShould, correctStringTest) {
    std::string jsonPath = __FILE__;
    jsonPath = jsonPath.substr(0, jsonPath.find("Jacobian/"));
    jsonPath += "Jacobian/tests/config/JacobianSmall.json";
    std::ifstream jsonConfigPath(jsonPath);
    nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);
    std::vector<double> lx = {0.0, 0.6127};
    std::vector<double> ly = {0.0, 0.11655};
    std::vector<double> lz = {0.1807, 0.0};

    ASSERT_NO_THROW(sut_.reset(new MathExprJacobian(jsonConfig, lx, ly, lz)));
    ASSERT_NO_THROW(logger_->info("Jacobian =\n{}", *sut_.get()));

    MathExprJacobian j(jsonConfig, lx, ly, lz);
    std::stringstream str;
    str << j;
    ASSERT_EQ(0, str.str().compare("J[0][0] = cos(q2) + l1x\n\nJ[0][1] = cos(q1) + l2x\n\n"
        "J[1][0] = q2\n\nJ[1][1] = sin(q1)\n"));
}
*/

TEST_F(JacobianShould, throwWrongTaskSpace) {
    EXPECT_THROW(
        sut_.reset(new KinChainJacobian(kinChainPlatformWrongTaskSpace_, taskSpacePlatformWrong_)),
            std::invalid_argument);
}

TEST_F(JacobianShould, returnCorrectResultKinematicChainCombined) {
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

    sut_.reset(new KinChainJacobian(kinChain_, taskSpace_));

    maxAbsErr_ = 1e-13;

    int chainSize = kinChain_->getChainSize();

    int numWheels = kinChain_->getNumWheels();

    // int taskSpaceDimension = kinChain_->getTaskSpaceDimension();
    int taskSpaceDimension = taskSpace_.dimension();

    int DoF = chainSize + numWheels;

    Eigen::MatrixXd expectedOut0(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut1(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut2(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut3(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut4(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut5(taskSpaceDimension, DoF);

    expectedOut0.block(0, 4, taskSpaceDimension, chainSize) << -0.539238162805340,
        0.379792404460527, -0.446859803849118, 0.325505687036145, 0.023008835505910, 0, 0,
        -0.000036326794865, -0.499999998680364, -0.000030229761683, -0.000004274488291, 0,
        0.030284594704168, 0.432779507040196, 1.297663523478877, -0.945540082065953,
        -0.066851203956180, 0, -0.000000000000000, -0.000068695318673, -0.945518573106741, 0,
        0.859833491194683, -0.393349627590407, 1.000000000000000, -0.999999997360728,
        0.000072653589730, 0, 0.416080767334026, 0.909357886348730, 0.000000000000000,
        -0.000023653695122, -0.325568153589457, 0, 0.295910733963289, -0.135441149618053;

    expectedOut1.block(0, 4, taskSpaceDimension, chainSize) << -30.399250121288727,
        30.504328933408466, 20.767961340213787, 0.076094994520411, 0.037709885484223, 0,
        -0.000000000000001, 0.002041851832578, 31.458416167111242, -0.946575800082101,
        -0.023141048816423, 0, 7.210661808772592, -7.682339948387561, 85.683480606760256,
        0.313374865788354, 0.055144776692305, 0, -0.000000000000000, 0.000070915157858,
        0.971854469413412, 0, -0.790903992312942, 0.299905643939045, 1.000000000000000,
        -0.999999997360728, 0.000065197431322, 0, 0.134014686456656, 0.935373877861575,
        0.000000000000000, 0.000015798243149, -0.235582015507278, 0, 0.597085369739851,
        0.187436158058435;

    expectedOut2.block(0, 4, taskSpaceDimension, chainSize) << 0.536608080940927,
        -0.132935580731264, 1.249711500521353, -0.987440110651557, -0.010220720581449, 0,
        -0.000000000000000, -0.000061871982119, 0.913598443445392, -0.072726921642118,
        -0.063158005617466, 0, 0.664602485005184, -0.941572455794119, 0.177385017682009,
        -0.140259840100086, -0.030084766863316, 0, -0.000000000000000, 0.000041104417445,
        0.140570533269043, 0, -0.097990040662644, 0.984624288607523, 1.000000000000000,
        -0.999999997360728, -0.000053535104363, 0, -0.415005807119498, -0.172490120372413, 0,
        0.000059908020891, -0.990070665311547, 0, 0.904526468373384, 0.027607402232708;

    expectedOut3.block(0, 4, taskSpaceDimension, chainSize) << 0.275797968615910,
        -0.526858797422411, 1.764457104713960, -0.982411975989561, -0.069470698099305, 0,
        0.000000000000000, 0.000121740631398, 2.426162578511434, 0.157937776378223,
        -0.012600321956484, 0, 2.805501657595053, -2.385268446170543, -0.178907232440846,
        0.099611084849976, -0.003680216323551, 0, -0.000000000000000, -0.000062370485690,
        -0.100936848584892, 0, 0.026259792057651, 0.183804125093733, 1.000000000000000,
        -0.999999997360728, 0.000043367310275, 0, -0.410942276296352, -0.894101009170803,
        -0.000000000000000, -0.000037262133801, -0.994892833785140, 0, 0.911283089316082,
        -0.408410858080781;

    expectedOut4.block(0, 4, taskSpaceDimension, chainSize) << -0.570551616260352,
        0.331613539538972, -0.476759816368956, 0.288126282617147, 0.039721269902054, 0, 0,
        0.000015002004245, -0.262411214882884, -0.373118244973213, 0.049777646332571, 0,
        -0.597195295001381, 0.169933030464149, -1.459138788158594, 0.881910437931956,
        0.030706622135260, 0, -0.000000000000000, 0.000063411485929, -0.950542767950550, 0,
        0.816164000643423, 0.135059818628002, 1.000000000000000, -0.999999997360728,
        -0.000071289489647, 0, -0.386078990576940, -0.596040206861025, -0.000000000000000,
        -0.000035461634944, 0.310593691524340, 0, -0.429906195685541, 0.791514319009605;

    expectedOut5.block(0, 4, taskSpaceDimension, chainSize) << 0.427947043987760,
        -0.566583475494876, 0.511646488867250, -0.412149195240936, -0.047966601809746, 0,
        0.000000000000000, 0.000027222979248, 0.774773059952499, -0.411665070588384,
        0.025399400390044, 0, -0.126270868206876, 0.595966623206796, 1.009185393317993,
        -0.812812961584465, -0.045305248820104, 0, 0.000000000000000, -0.000069681612100,
        -0.891935988299741, 0, 0.733537045005377, -0.040263151291035, 1.000000000000000,
        -0.999999997360728, 0.000052851757410, 0, 0.379146567825571, 0.852777797392518,
        0.000000000000000, -0.000020567377951, 0.452161685663941, 0, -0.564066736930011,
        0.520719604895457;

    expectedOut0.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.059006211180124224,
        -0.059006211180124224, -0.059006211180124224, 0.059006211180124224;

    expectedOut1.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.059006211180124224,
        -0.059006211180124224, -0.059006211180124224, 0.059006211180124224;

    expectedOut2.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.059006211180124224,
        -0.059006211180124224, -0.059006211180124224, 0.059006211180124224;

    expectedOut3.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.059006211180124224,
        -0.059006211180124224, -0.059006211180124224, 0.059006211180124224;

    expectedOut4.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.059006211180124224,
        -0.059006211180124224, -0.059006211180124224, 0.059006211180124224;

    expectedOut5.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.059006211180124224,
        -0.059006211180124224, -0.059006211180124224, 0.059006211180124224;

    Eigen::MatrixXd out0(taskSpaceDimension, DoF);
    Eigen::MatrixXd out1(taskSpaceDimension, DoF);
    Eigen::MatrixXd out2(taskSpaceDimension, DoF);
    Eigen::MatrixXd out3(taskSpaceDimension, DoF);
    Eigen::MatrixXd out4(taskSpaceDimension, DoF);
    Eigen::MatrixXd out5(taskSpaceDimension, DoF);

    out0 = sut_->evaluate(jointPositions0);
    out1 = sut_->evaluate(jointPositions1);
    out2 = sut_->evaluate(jointPositions2);
    out3 = sut_->evaluate(jointPositions3);
    out4 = sut_->evaluate(jointPositions4);
    out5 = sut_->evaluate(jointPositions5);

    EXPECT_TRUE((out0 - expectedOut0).norm() < maxAbsErr_);
    EXPECT_TRUE((out1 - expectedOut1).norm() < maxAbsErr_);
    EXPECT_TRUE((out2 - expectedOut2).norm() < maxAbsErr_);
    EXPECT_TRUE((out3 - expectedOut3).norm() < maxAbsErr_);
    EXPECT_TRUE((out4 - expectedOut4).norm() < maxAbsErr_);
    EXPECT_TRUE((out5 - expectedOut5).norm() < maxAbsErr_);

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out0 - expectedOut0).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out1 - expectedOut1).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out2 - expectedOut2).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out3 - expectedOut3).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out4 - expectedOut4).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out5 - expectedOut5).norm() << std::endl;

    //  std::cout << std::endl
    //           << std::endl;
    // std::cout << out0 << std::endl;
}

TEST_F(JacobianShould, returnCorrectResultKinematicChainArm) {
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

    sut_.reset(new KinChainJacobian(kinChainArm_, taskSpaceArm_));

    maxAbsErr_ = 1e-13;

    int chainSize = kinChainArm_->getChainSize();

    int numWheels = kinChainArm_->getNumWheels();

    // int taskSpaceDimension = kinChainArm_->getTaskSpaceDimension();

    int taskSpaceDimension = taskSpaceArm_.dimension();

    int DoF = chainSize + numWheels;

    Eigen::MatrixXd expectedOut0(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut1(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut2(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut3(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut4(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut5(taskSpaceDimension, DoF);

    expectedOut0.block(0, 0, taskSpaceDimension, chainSize) << -3.704966695014239,
        0.266218714266114, 0.936350912387054, -0.126200905765627, 0.802924200338383, 0,
        1.933673876669431, -1.641587967672526, 0.350294119272061, 1.362181345307387,
        0.398831271591907, 0, 0.000000000000000, 0.498265385713240, 0.023259382522801,
        0.975482320471923, -0.442997003727283, 0, 0, 0.975170327201816, 0, 0.995244536759705, 0,
        -0.547242581865933, 0.000000000000000, 0.097843395007256, 0, 0.024736814594452, 0,
        0.787826947091429, 1.000000000000000, -0.198669330795061, 0, 0.094214659441511, 0,
        -0.282584957188643;

    expectedOut1.block(0, 0, taskSpaceDimension, chainSize) << 5.693997339292743,
        -0.600318051209301, -0.850537383273120, 0.650044968359246, -0.746313124750729, 0,
        -2.995573913424477, 0.946412295919792, -0.514567305596923, -2.100774361827737,
        -0.389386907676351, 0, -0.000000000000000, 1.910618508354144, -0.108658398964996,
        0.598473693748087, -0.539809740515167, 0, 0.000000000000000, -0.958039156834720, 0,
        -0.958266799958099, 0, 0.662789836816290, 0.000000000000000, -0.206619144739448, 0,
        -0.268765548106706, 0, -0.509149667136572, 1.000000000000000, -0.198669330795061, 0,
        0.097415708430237, 0, -0.549068528207320;

    expectedOut2.block(0, 0, taskSpaceDimension, chainSize) << -0.097888291161730,
        -1.655320647130605, -0.715211518303331, 0.755509171300742, -0.497101213393807, 0,
        -6.030835410395191, -2.767233275960585, 0.476336280408626, -0.538141432434725,
        0.794854229048838, 0, -0.000000000000000, 0.019952662717383, -0.511445238566866,
        -2.336628068033364, -0.347990140960318, 0, -0.000000000000000, -0.841702863515605, 0,
        -0.903896920901401, 0, 0.853121043893394, 0, 0.502062532510715, 0, 0.247089667597319, 0,
        0.374534401041833, 1.000000000000000, -0.198669330795061, 0, -0.349166224815075, 0,
        -0.363192052366898;

    expectedOut3.block(0, 0, taskSpaceDimension, chainSize) << 0.525774222960142,
        -0.932061003835985, 0.982294929967924, -0.698113227343159, 0.624109489898737, 0,
        -1.103651543932281, -1.934640670274767, -0.125658904082239, -1.504531059745078,
        -0.609297213225141, 0, -0.000000000000000, -3.553288762411979, 0.138947869304147,
        -0.963335568999583, 0.489126006847332, 0, -0.000000000000000, 0.974470254582337, 0,
        0.922617945673894, 0, -0.780287418361985, -0.000000000000000, -0.104585945210996, 0,
        -0.377547834537221, 0, -0.518470790572437, 1.000000000000000, -0.198669330795061, 0,
        -0.078954157311309, 0, 0.349770759311267;

    expectedOut4.block(0, 0, taskSpaceDimension, chainSize) << 4.421755285163838, 1.945129681798664,
        -0.682246667000096, -0.770075047229976, -0.238824518634351, 0, 1.325207134019954,
        -1.585358793231741, -0.582302419860597, 0.238488936092845, -0.953370008187516, 0, 0,
        0.675107137834956, -0.442112403345295, 0.288693615673277, 0.184522293470482, 0,
        -0.000000000000000, -0.576831909767679, 0, -0.414452751742042, 0, -0.149487194230338,
        -0.000000000000000, -0.792335437094173, 0, -0.788791760393573, 0, 0.223851141455148,
        1.000000000000000, -0.198669330795061, 0, -0.453912409291327, 0, 0.963090984918024;

    expectedOut5.block(0, 0, taskSpaceDimension, chainSize) << -3.635009240050087,
        0.234302981139969, 0.862658370289330, -0.277107190765593, 0.903159216127200, 0,
        2.888858660319317, -1.339109175189953, 0.462089270525729, 1.343899518134876,
        -0.256898075353348, 0, 0.000000000000000, 0.191882985112724, -0.205655153679057,
        -1.759903485747716, -0.343957568900944, 0, -0.000000000000000, 0.969838234218457, 0,
        0.956153524282262, 0, -0.417299533674564, 0.000000000000000, 0.141224277124961, 0,
        0.285067964838836, 0, -0.337184298314023, 1.000000000000000, -0.198669330795061, 0,
        0.067131910633115, 0, -0.843900378104828;

    Eigen::MatrixXd out0(taskSpaceDimension, DoF);
    Eigen::MatrixXd out1(taskSpaceDimension, DoF);
    Eigen::MatrixXd out2(taskSpaceDimension, DoF);
    Eigen::MatrixXd out3(taskSpaceDimension, DoF);
    Eigen::MatrixXd out4(taskSpaceDimension, DoF);
    Eigen::MatrixXd out5(taskSpaceDimension, DoF);

    out0 = sut_->evaluate(jointPositions0);
    out1 = sut_->evaluate(jointPositions1);
    out2 = sut_->evaluate(jointPositions2);
    out3 = sut_->evaluate(jointPositions3);
    out4 = sut_->evaluate(jointPositions4);
    out5 = sut_->evaluate(jointPositions5);

    EXPECT_TRUE((out0 - expectedOut0).norm() < maxAbsErr_);
    EXPECT_TRUE((out1 - expectedOut1).norm() < maxAbsErr_);
    EXPECT_TRUE((out2 - expectedOut2).norm() < maxAbsErr_);
    EXPECT_TRUE((out3 - expectedOut3).norm() < maxAbsErr_);
    EXPECT_TRUE((out4 - expectedOut4).norm() < maxAbsErr_);
    EXPECT_TRUE((out5 - expectedOut5).norm() < maxAbsErr_);

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out0 - expectedOut0).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out1 - expectedOut1).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out2 - expectedOut2).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out3 - expectedOut3).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out4 - expectedOut4).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out5 - expectedOut5).norm() << std::endl;

    //  std::cout << std::endl
    //           << std::endl;
    // std::cout << out0 << std::endl;
}

TEST_F(JacobianShould, returnCorrectResultKinematicChainPlatform) {
    crf::utility::types::JointPositions jointPositions0({0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions jointPositions1(
        {-0.029005763708726, 0.182452167505983, -1.565056014150725, -0.084539479817724});
    crf::utility::types::JointPositions jointPositions2({0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions jointPositions3({0.0, 5.0, 4.0, -2.0});
    crf::utility::types::JointPositions jointPositions4({0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions jointPositions5({-0.7581, 2.0783, -2.2220, 0.4488});

    sut_.reset(new KinChainJacobian(kinChainPlatform_, taskSpacePlatform_));

    maxAbsErr_ = 1e-13;

    int chainSize = kinChainPlatform_->getChainSize();

    int numWheels = kinChainPlatform_->getNumWheels();

    // int taskSpaceDimension = kinChainPlatform_->getTaskSpaceDimension();

    int taskSpaceDimension = taskSpacePlatform_.dimension();

    int DoF = chainSize + numWheels;

    Eigen::MatrixXd expectedOut0(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut1(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut2(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut3(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut4(taskSpaceDimension, DoF);
    Eigen::MatrixXd expectedOut5(taskSpaceDimension, DoF);

    expectedOut0.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0.059006211180124224, -0.059006211180124224, -0.059006211180124224,
        0.059006211180124224;

    expectedOut1.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0.059006211180124224, -0.059006211180124224, -0.059006211180124224,
        0.059006211180124224;

    expectedOut2.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0.059006211180124224, -0.059006211180124224, -0.059006211180124224,
        0.059006211180124224;

    expectedOut3.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0.059006211180124224, -0.059006211180124224, -0.059006211180124224,
        0.059006211180124224;

    expectedOut4.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0.059006211180124224, -0.059006211180124224, -0.059006211180124224,
        0.059006211180124224;

    expectedOut5.block(0, 0, taskSpaceDimension, numWheels) << 0.019, 0.019, 0.019, 0.019, 0.019,
        -0.019, 0.019, -0.019, 0.059006211180124224, -0.059006211180124224, -0.059006211180124224,
        0.059006211180124224;

    Eigen::MatrixXd out0(taskSpaceDimension, DoF);
    Eigen::MatrixXd out1(taskSpaceDimension, DoF);
    Eigen::MatrixXd out2(taskSpaceDimension, DoF);
    Eigen::MatrixXd out3(taskSpaceDimension, DoF);
    Eigen::MatrixXd out4(taskSpaceDimension, DoF);
    Eigen::MatrixXd out5(taskSpaceDimension, DoF);

    out0 = sut_->evaluate(jointPositions0);
    out1 = sut_->evaluate(jointPositions1);
    out2 = sut_->evaluate(jointPositions2);
    out3 = sut_->evaluate(jointPositions3);
    out4 = sut_->evaluate(jointPositions4);
    out5 = sut_->evaluate(jointPositions5);

    EXPECT_TRUE((out0 - expectedOut0).norm() < maxAbsErr_);
    EXPECT_TRUE((out1 - expectedOut1).norm() < maxAbsErr_);
    EXPECT_TRUE((out2 - expectedOut2).norm() < maxAbsErr_);
    EXPECT_TRUE((out3 - expectedOut3).norm() < maxAbsErr_);
    EXPECT_TRUE((out4 - expectedOut4).norm() < maxAbsErr_);
    EXPECT_TRUE((out5 - expectedOut5).norm() < maxAbsErr_);

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out0 - expectedOut0).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out1 - expectedOut1).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out2 - expectedOut2).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out3 - expectedOut3).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out4 - expectedOut4).norm() << std::endl;

    // std::cout << std::endl
    //           << std::endl;
    // std::cout << (out5 - expectedOut5).norm() << std::endl;

    //  std::cout << std::endl
    //           << std::endl;
    // std::cout << out0 << std::endl;
}
