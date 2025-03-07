/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 * 
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "InverseKinematics/OptOLIK/OptOLIK.hpp"
#include "InverseKinematics/JointLimits/JointLimits.hpp"

using crf::control::inversekinematics::IOpenLoopInverseKinematics;
using crf::control::inversekinematics::OptOLIK;

class OptOLIKShould: public ::testing::Test {
 protected:
    OptOLIKShould(): logger_("OptOLIKShould"),
        diagW_(),
        objFunVector_(),
        kinManip0_(0.0),
        alpha0_(0.0) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("InverseKinematics"));
        testDirName_ += "InverseKinematics/tests/config/";
    }
    ~OptOLIKShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        std::ifstream jsonConfigPath(testDirName_ + "UR10eSimFKLengthsJacobianTest.json");
        nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);
        configuration_ = std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfig);

        diagW_.resize(6);
        diagW_ = {500, 500, 500, 0.05, 0.05, 0.05};
        objFunVector_.resize(0);
        kinManip0_ = 0.001;
        alpha0_ = 0.001;
    }

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration_;
    std::vector<double> diagW_;
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector_;  // NOLINT
    double kinManip0_, alpha0_;
    std::unique_ptr<IOpenLoopInverseKinematics> optOLIK_;
};

TEST_F(OptOLIKShould, ThrowVariablesAreNotCorrectlyInitialized) {
    // Error in lenghts
    std::ifstream jsonConfigPathError(testDirName_ +
        "UR10eSimFKLengthsJacobianTestV2LengthsError.json");
    nlohmann::json jsonConfigError = nlohmann::json::parse(jsonConfigPathError);
    ASSERT_THROW(std::shared_ptr<crf::actuators::robot::RobotConfiguration> configurationError =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfigError),
        std::invalid_argument);

    // kinManip0, alpha0 are 0 or negative
    double kinManip0Error(-0.003);
    ASSERT_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW_, objFunVector_,
        kinManip0Error, alpha0_)), std::invalid_argument);
    double alpha0Error(0.0);
    ASSERT_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW_, objFunVector_,
        kinManip0_, alpha0Error)), std::invalid_argument);

    // Wrong size diagW
    std::vector<double> diagW({500, 500, 500, 0.05, 0.05});
    ASSERT_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW, objFunVector_,
        kinManip0_, alpha0_)), std::invalid_argument);
    // Wrong elements inside diagW
    std::vector<double> diagW2({0.0, -1.0, 0.0, -1.0, 0.0, -1.0});
    ASSERT_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW2, objFunVector_,
        kinManip0_, alpha0_)), std::invalid_argument);
}

TEST_F(OptOLIKShould, ThrowEmptyJSONExpression) {
    // Jacobian configuration is empty
    std::ifstream jsonConfigPathError(testDirName_ +
        "UR10eSimFKLengthsJacobianTestV3EmptyJacobian.json");
    nlohmann::json jsonConfigError = nlohmann::json::parse(jsonConfigPathError);
    ASSERT_THROW(std::shared_ptr<crf::actuators::robot::RobotConfiguration> configurationError =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfigError),
        std::invalid_argument);
}

TEST_F(OptOLIKShould, returnFalseIfInputDimensionQIsDifferentThanTheExpected) {
    ASSERT_NO_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW_, objFunVector_,
        kinManip0_, alpha0_)));

    crf::utility::types::JointPositions qAttrError({0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions q({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_THROW(optOLIK_->getResults(qAttrError, q, TaskPose(), zd), std::invalid_argument);
    ASSERT_THROW(optOLIK_->getExtendedResults(qAttrError, q, TaskPose(), zd),
        std::invalid_argument);

    qAttrError = crf::utility::types::JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    q = crf::utility::types::JointPositions({0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_THROW(optOLIK_->getResults(qAttrError, q, TaskPose(), zd), std::invalid_argument);
    ASSERT_THROW(optOLIK_->getExtendedResults(qAttrError, q, TaskPose(), zd),
        std::invalid_argument);
}

TEST_F(OptOLIKShould, returnCorrectResults) {
    crf::utility::types::JointPositions qAttr({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});


    // Success
    crf::utility::types::JointVelocities qdResult1({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_NO_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW_, objFunVector_,
        kinManip0_, alpha0_)));

    crf::utility::types::JointPositions q1(
        {15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180});

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> result1(  // NOLINT
        optOLIK_->getResults(qAttr, q1, TaskPose(), zd));
    ASSERT_TRUE(areAlmostEqual(std::get<0>(result1), JointPositions(1)));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_NEAR(std::get<1>(result1)[i], qdResult1[i], 1e-11);
    }
    ASSERT_TRUE(areAlmostEqual(std::get<2>(result1), JointAccelerations(1)));
    ASSERT_EQ(std::get<3>(result1), crf::control::inversekinematics::ResultFlags::success);

    crf::control::inversekinematics::ResultsIK extendedResults1(
        optOLIK_->getExtendedResults(qAttr, q1, TaskPose(), zd));
    ASSERT_TRUE(areAlmostEqual(extendedResults1.zDesired(), TaskPose()));
    ASSERT_TRUE(areAlmostEqual(extendedResults1.zddDesired(), TaskAcceleration()));
    ASSERT_TRUE(areAlmostEqual(extendedResults1.qResult(), JointPositions(1)));
    ASSERT_TRUE(areAlmostEqual(extendedResults1.qddResult(), JointAccelerations(1)));
    ASSERT_EQ(extendedResults1.zError(), std::vector<double>());
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_NEAR(extendedResults1.zdDesired()[i], zd[i], 1e-13);
        ASSERT_NEAR(extendedResults1.qdResult()[i], qdResult1[i], 1e-11);
        ASSERT_DOUBLE_EQ(extendedResults1.penaltyGradients()(i, 0), 0.0);
    }
    ASSERT_DOUBLE_EQ(extendedResults1.kinematicManipulability(), 0.023506349416319277);
    ASSERT_EQ(extendedResults1.flag(), crf::control::inversekinematics::ResultFlags::success);


    // Low Manipulability
    crf::utility::types::JointVelocities qdResult2({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_NO_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW_, objFunVector_,
        kinManip0_, alpha0_)));

    crf::utility::types::JointPositions q2(
        {5*M_PI/180, -85*M_PI/180, 5*M_PI/180, -85*M_PI/180, 5*M_PI/180, 5*M_PI/180});

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> result2(  // NOLINT
        optOLIK_->getResults(qAttr, q2, TaskPose(), zd));
    ASSERT_TRUE(areAlmostEqual(std::get<0>(result2), JointPositions(1)));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_NEAR(std::get<1>(result2)[i], qdResult2[i], 1e-11);
    }
    ASSERT_TRUE(areAlmostEqual(std::get<2>(result2), JointAccelerations(1)));
    ASSERT_EQ(std::get<3>(result2),
        crf::control::inversekinematics::ResultFlags::lowManipulability);


    // Workspace Violation
    ASSERT_NO_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW_, objFunVector_,
        kinManip0_, alpha0_)));

    crf::utility::types::JointPositions q4({std::nan(""), -104.78*M_PI/180, 47.51*M_PI/180,
        -107.73*M_PI/180, 5*M_PI/180, 5*M_PI/180});

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> result4(  // NOLINT
        optOLIK_->getResults(qAttr, q4, TaskPose(), zd));
    ASSERT_TRUE(areAlmostEqual(std::get<0>(result4), JointPositions(1)));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_TRUE(std::isnan(std::get<1>(result4)[i]));
    }
    ASSERT_TRUE(areAlmostEqual(std::get<2>(result4), JointAccelerations(1)));
    ASSERT_EQ(std::get<3>(result4),
        crf::control::inversekinematics::ResultFlags::workspaceViolation);
}

TEST_F(OptOLIKShould, checkBehaviorWithAnObjectiveFunction) {
    // Objective Function
    double rangeSinusoid = 2.0;
    double cycleTime = 0.002;
    double c = 0.1;
    double p = 10.2;
    crf::utility::types::JointPositions minLimit({-2.0, -2.0, -2.0, -2.0, -2.0, std::nan("")});
    crf::utility::types::JointPositions maxLimit({2.0, 2.0, 2.0, 2.0, 2.0, std::nan("")});
    std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction> jointLimits =
        std::make_unique<crf::control::inversekinematics::JointLimits>(
        rangeSinusoid, cycleTime, c, p, minLimit, maxLimit);
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector(1);  // NOLINT
    objFunVector[0] = jointLimits;

    // OptCLIK
    crf::utility::types::JointVelocities qdResult({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_NO_THROW(optOLIK_.reset(new OptOLIK(configuration_, diagW_, objFunVector_,
        kinManip0_, alpha0_)));

    crf::utility::types::JointPositions qAttr({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions q(
        {15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180});
    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> result(  // NOLINT
        optOLIK_->getResults(qAttr, q, TaskPose(), zd));
    ASSERT_TRUE(areAlmostEqual(std::get<0>(result), JointPositions(1)));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_NEAR(std::get<1>(result)[i], qdResult[i], 1e-11);
    }
    ASSERT_TRUE(areAlmostEqual(std::get<2>(result), JointAccelerations(1)));
    ASSERT_EQ(std::get<3>(result), crf::control::inversekinematics::ResultFlags::success);
}

TEST_F(OptOLIKShould, returnCorrectResultsInReducedMode) {
    // Correct
    std::ifstream jsonConfigPath(testDirName_ +
        "UR10eSimFKLengthsJacobianTestV4ReducedCorrect.json");
    nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfig);

    std::vector<double> diagW({500, 500, 0.05, 0.05});
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector;  // NOLINT
    objFunVector.resize(0);
    double kinManip0 = 0.001, alpha0 = 0.001;

    optOLIK_.reset(new OptOLIK(configuration, diagW, objFunVector,
        kinManip0, alpha0));

    crf::utility::types::JointPositions qAttr({0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions q({15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180});
    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    ASSERT_NO_THROW(optOLIK_->getResults(qAttr, q, TaskPose(), zd));
}
