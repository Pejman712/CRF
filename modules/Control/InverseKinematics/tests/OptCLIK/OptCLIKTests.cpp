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

#include "InverseKinematics/OptCLIK/OptCLIK.hpp"
#include "InverseKinematics/JointLimits/JointLimits.hpp"

using crf::control::inversekinematics::IClosedLoopInverseKinematics;
using crf::control::inversekinematics::OptCLIK;

class OptCLIKShould: public ::testing::Test {
 protected:
    OptCLIKShould(): logger_("OptCLIKShould"),
        qInitial_(1),
        time_(0),
        cardanTolerance_(),
        diagW_(),
        objFunVector_(),
        K_(0.0),
        kinManip0_(0.0),
        alpha0_(0.0) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("InverseKinematics"));
        testDirName_ += "InverseKinematics/tests/config/";
    }
    ~OptCLIKShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        qInitial_ = crf::utility::types::JointPositions(
            {15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180});
        time_ = std::chrono::microseconds(200);

        std::ifstream jsonConfigPath(testDirName_ + "UR10eSimFKLengthsJacobianTest.json");
        nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);
        configuration_ = std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfig);

        cardanTolerance_ = crf::utility::types::TaskPose({0.01, 0.01, 0.01},
            crf::math::rotation::CardanXYZ({2*M_PI/180, 2*M_PI/180, 2*M_PI/180}));


        diagW_.resize(6);
        diagW_ = {500, 500, 500, 0.05, 0.05, 0.05};
        objFunVector_.resize(0);
        K_ = 500;
        kinManip0_ = 0.001;
        alpha0_ = 0.001;
    }

    crf::utility::logger::EventLogger logger_;
    crf::utility::types::JointPositions qInitial_;
    std::string testDirName_;
    std::chrono::microseconds time_;
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration_;
    crf::utility::types::TaskPose cardanTolerance_;
    std::vector<double> diagW_;
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector_;  // NOLINT
    double K_, kinManip0_, alpha0_;
    std::unique_ptr<IClosedLoopInverseKinematics> optCLIK_;
};

TEST_F(OptCLIKShould, ThrowVariablesAreNotCorrectlyInitialized) {
    // Error in qInitial size
    crf::utility::types::JointPositions qInitialError(
        {5*M_PI/180, 5*M_PI/180, 5*M_PI/180, 5*M_PI/180, 5*M_PI/180});
    ASSERT_THROW(optCLIK_.reset(new OptCLIK(qInitialError, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)), std::invalid_argument);

    // Error in lenghts
    std::ifstream jsonConfigPathError(testDirName_ +
        "UR10eSimFKLengthsJacobianTestV2LengthsError.json");
    nlohmann::json jsonConfigError = nlohmann::json::parse(jsonConfigPathError);
    ASSERT_THROW(std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfigError),
        std::invalid_argument);

    // Time 0 or negative
    std::chrono::microseconds time(0);
    ASSERT_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time, configuration_, diagW_, objFunVector_,
        cardanTolerance_, K_, kinManip0_, alpha0_)), std::invalid_argument);
    time = std::chrono::microseconds(-200);
    ASSERT_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time, configuration_, diagW_, objFunVector_,
        cardanTolerance_, K_, kinManip0_, alpha0_)), std::invalid_argument);

    // K, kinManip0, alpha0 are 0 or negative
    double KError(0.0);
    ASSERT_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_, objFunVector_,
        cardanTolerance_, KError, kinManip0_, alpha0_)), std::invalid_argument);
    double kinManip0Error(-0.003);
    ASSERT_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_, objFunVector_,
        cardanTolerance_, K_, kinManip0Error, alpha0_)), std::invalid_argument);
    double alpha0Error(0.0);
    ASSERT_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_, objFunVector_,
        cardanTolerance_, K_, kinManip0_, alpha0Error)), std::invalid_argument);

    // Wrong size diagW
    std::vector<double> diagW({500, 500, 500, 0.05, 0.05});
    ASSERT_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW, objFunVector_,
        cardanTolerance_, K_, kinManip0_, alpha0_)), std::invalid_argument);
}

TEST_F(OptCLIKShould, ThrowEmptyJSONExpression) {
    // Forward Kinematics configuration is empty
    std::ifstream jsonConfigPathError1(testDirName_ +
        "UR10eSimFKLengthsJacobianTestV3EmptyFK.json");
    nlohmann::json jsonConfigError1 = nlohmann::json::parse(jsonConfigPathError1);
    ASSERT_THROW(std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfigError1),
        std::invalid_argument);

    // Forward Kinematics NoPositionFK
    std::ifstream jsonConfigPathError2(testDirName_ +
        "UR10eSimFKLengthsJacobianTestV3NoPositionFK.json");
    nlohmann::json jsonConfigError2 = nlohmann::json::parse(jsonConfigPathError2);
    auto configurationError2 = std::make_shared<crf::actuators::robot::RobotConfiguration>(
        jsonConfigError2);
    ASSERT_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configurationError2,
        diagW_, objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)), std::invalid_argument);

    // Jacobian configuration is empty
    std::ifstream jsonConfigPathError3(testDirName_ +
        "UR10eSimFKLengthsJacobianTestV3EmptyJacobian.json");
    nlohmann::json jsonConfigError3 = nlohmann::json::parse(jsonConfigPathError3);
    ASSERT_THROW(std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfigError3),
        std::invalid_argument);
}

TEST_F(OptCLIKShould, returnFalseIfInputDimensionQIsDifferentThanTheExpected) {
    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)));

    crf::utility::types::TaskPose z(Eigen::Vector3d({0.30459577, -0.14780326, 1.4725429}),
        Eigen::AngleAxisd(0.8951107, Eigen::Vector3d({0.01944744, 0.44541964, 0.3900936})));
    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions qAttrError({0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_THROW(optCLIK_->getResults(qAttrError, z, zd), std::invalid_argument);
    ASSERT_THROW(optCLIK_->getExtendedResults(qAttrError, z, zd), std::invalid_argument);
}

TEST_F(OptCLIKShould, returnCorrectResults) {
    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions qAttr({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});


    // Success
    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)));

    crf::utility::types::TaskPose z1(
        Eigen::Vector3d({-0.9143011906611436, -0.5418296124948834, -0.3697303070650604}),
        crf::math::rotation::CardanXYZ(
            {-1.2238988279449308, -2.1392167582457033, 2.9204082579391146}));

    crf::utility::types::JointPositions qResult1(
        {0.2617993877991494, 0.2617993877991494, 0.2617993877991494,
        0.2617993877991494, 0.2617993877991494, 0.2617993877991494});
    crf::utility::types::JointPositions qdResult1({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> resultCL1(  // NOLINT
        optCLIK_->getResults(qAttr, z1, zd));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_DOUBLE_EQ(std::get<0>(resultCL1)[i], qResult1[i]);
        ASSERT_NEAR(std::get<1>(resultCL1)[i], qdResult1[i], 1e-11);
    }
    ASSERT_EQ(std::get<3>(resultCL1), crf::control::inversekinematics::ResultFlags::success);

    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)));
    std::array<double, 6> zErrorResult({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::vector<double> penaltyGradients({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::control::inversekinematics::ResultsIK extendedResultsCL1(
        optCLIK_->getExtendedResults(qAttr, z1, zd));
    ASSERT_TRUE(areAlmostEqual(extendedResultsCL1.zddDesired(), TaskAcceleration()));
    ASSERT_TRUE(areAlmostEqual(extendedResultsCL1.qddResult(), JointAccelerations(1)));
    ASSERT_TRUE(areAlmostEqual(extendedResultsCL1.zDesired(), z1));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_NEAR(extendedResultsCL1.zdDesired()[i], zd[i], 1e-13);
        ASSERT_DOUBLE_EQ(extendedResultsCL1.qResult()[i], qResult1[i]);
        ASSERT_NEAR(extendedResultsCL1.qdResult()[i], qdResult1[i], 1e-11);
        ASSERT_NEAR(extendedResultsCL1.zError()[i], zErrorResult[i], 1e-15);
        ASSERT_DOUBLE_EQ(extendedResultsCL1.penaltyGradients()(i, 0), penaltyGradients[i]);
    }
    ASSERT_DOUBLE_EQ(extendedResultsCL1.kinematicManipulability(), 0.023506349416319277);
    ASSERT_EQ(extendedResultsCL1.flag(), crf::control::inversekinematics::ResultFlags::success);


    // Low Manipulability
    crf::utility::types::JointPositions qInitial2(
        {5*M_PI/180, -85*M_PI/180, 5*M_PI/180, -85*M_PI/180, 5*M_PI/180, 5*M_PI/180});
    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial2, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)));

    crf::utility::types::TaskPose z2(
        Eigen::Vector3d({-0.14789742906142567, -0.30430457311567827, 1.4723306573661725}),
        Eigen::AngleAxisd(2.7720648451782917,
            Eigen::Vector3d({ 0.06269551620516657, 0.7084218261637105, -0.7029991383086049})));

    crf::utility::types::JointPositions qResult2(
        {0.08726646259971647, -1.4835298641951802, 0.08726646259971647,
        -1.4835298641951802, 0.08726646259971647, 0.08726646259971647});
    crf::utility::types::JointVelocities qdResult2({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> resultCL2(  // NOLINT
        optCLIK_->getResults(qAttr, z2, zd));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_DOUBLE_EQ(std::get<0>(resultCL2)[i], qResult2[i]);
        ASSERT_NEAR(std::get<1>(resultCL2)[i], qdResult2[i], 1e-11);
    }
    ASSERT_EQ(std::get<3>(resultCL2),
        crf::control::inversekinematics::ResultFlags::lowManipulability);


    // End effector Tolerance Violation
    crf::utility::types::JointPositions qInitial4({5*M_PI/180, -104.78*M_PI/180, 47.51*M_PI/180,
        -107.73*M_PI/180, 5*M_PI/180, 5*M_PI/180});
    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial4, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)));

    crf::utility::types::TaskPose z4(Eigen::Vector3d({0.30459577, -0.14780326, 1.4725429}),
        Eigen::AngleAxisd(0.3900936, Eigen::Vector3d({0.8951107, 0.01944744, 0.44541964})));

    // crf::utility::types::JointPositions qResult4(
    //     {0.024712750111234305, -1.8612582632868833, 0.83377432550717723,
    //     -2.050387871022866, 0.16276764773148261, 0.30780969079597709});
    crf::utility::types::JointPositions qResult4(
        {0.026992196232883937, -1.854461386577966, 0.83619200673900496,
        -2.0884740029365658, 0.20179713108334157, 0.3961492109169365});
    // crf::utility::types::JointPositions qdResult4(
    //     {-312.76856244241083, -162.5113652361228, 22.841989423356743,
    //     -850.72333924687427, 377.50592565883062, 1102.7161409813029});
    crf::utility::types::JointPositions qdResult4(
        {-301.37133183416267, -128.52698169153686, 34.930395582495294,
        -1041.1539988153741, 572.65334241812548, 1544.4137415861001});

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> resultCL4(  // NOLINT
        optCLIK_->getResults(qAttr, z4, zd));
    for (unsigned int i = 0; i < 6; i++) {
        EXPECT_DOUBLE_EQ(std::get<0>(resultCL4)[i], qResult4[i]);
        EXPECT_NEAR(std::get<1>(resultCL4)[i], qdResult4[i], 1e-11);
    }
    ASSERT_EQ(std::get<3>(resultCL4),
        crf::control::inversekinematics::ResultFlags::endEffectorToleranceViolation);


    // Workspace Violation
    crf::utility::types::JointPositions qInitial5({5*M_PI/180, -104.78*M_PI/180, 47.51*M_PI/180,
        -107.73*M_PI/180, 5*M_PI/180, 5*M_PI/180});
    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial5, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)));

    crf::utility::types::TaskPose z5(Eigen::Vector3d({std::nan(""), -0.14780326, 1.4725429}),
        Eigen::AngleAxisd(0.3900936, Eigen::Vector3d({0.01944744, 0.44541964, 0.8951107})));

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> resultCL5(  // NOLINT
        optCLIK_->getResults(qAttr, z5, zd));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_TRUE(std::isnan(std::get<0>(resultCL5)[i]));
        ASSERT_TRUE(std::isnan(std::get<1>(resultCL5)[i]));
    }
    ASSERT_EQ(std::get<3>(resultCL5),
        crf::control::inversekinematics::ResultFlags::workspaceViolation);
}

TEST_F(OptCLIKShould, checkBehaviorWithAnObjectiveFunction) {
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
    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_,
        objFunVector, cardanTolerance_, K_, kinManip0_, alpha0_)));

    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions qAttr({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    crf::utility::types::TaskPose z(
        Eigen::Vector3d({-0.9143011906611436, -0.5418296124948834, -0.3697303070650604}),
        crf::math::rotation::CardanXYZ(
            {-1.2238988279449308, -2.1392167582457033, 2.9204082579391146}));
    crf::utility::types::JointPositions qResult(
        {0.2617993877991494, 0.2617993877991494, 0.2617993877991494,
        0.2617993877991494, 0.2617993877991494, 0.2617993877991494});
    crf::utility::types::JointPositions qdResult({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> resultCL(  // NOLINT
        optCLIK_->getResults(qAttr, z, zd));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_DOUBLE_EQ(std::get<0>(resultCL)[i], qResult[i]);
        ASSERT_NEAR(std::get<1>(resultCL)[i], qdResult[i], 1e-11);
    }
    ASSERT_EQ(std::get<3>(resultCL), crf::control::inversekinematics::ResultFlags::success);
}

TEST_F(OptCLIKShould, returnCorrectResultsInReducedMode) {
    // Correct
    std::ifstream jsonConfigPath(testDirName_ +
        "UR10eSimFKLengthsJacobianTestV4ReducedCorrect.json");
    nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> configuration =
        std::make_shared<crf::actuators::robot::RobotConfiguration>(jsonConfig);

    crf::utility::types::JointPositions qInitial(
        {15*M_PI/180, 15*M_PI/180, 15*M_PI/180, 15*M_PI/180});
    std::chrono::microseconds time = std::chrono::microseconds(200);
    crf::utility::types::TaskPose tolerance(Eigen::Vector3d({0.01, 0.01, 2*M_PI/180}),
        crf::math::rotation::CardanXYZ({0.0, 2*M_PI/180, 0.0}));
    std::vector<double> diagW({500, 500, 0.05, 0.05});
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>> objFunVector;  // NOLINT
    objFunVector.resize(0);
    double K = 500, kinManip0 = 0.001, alpha0 = 0.001;

    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial, time, configuration, diagW, objFunVector,
        tolerance, K, kinManip0, alpha0)));

    crf::utility::types::JointPositions qAttr({0.0, 0.0, 0.0, 0.0});
    crf::utility::types::TaskPose z(Eigen::Vector3d(
        {-0.9143011906611436, -0.5418296124948834, -0.3697303070650604}),
        crf::math::rotation::CardanXYZ({0.0, -1.2238988279449308, 0.0}));
    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    ASSERT_NO_THROW(optCLIK_->getResults(qAttr, z, zd));
}

TEST_F(OptCLIKShould, throwWhenInputIsWrongInUpdateInitialJointPositionsFunction) {
    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)));

    JointPositions qActualWrong({0.0, 3.9});
    ASSERT_THROW(optCLIK_->updateInitialJointPositions(qActualWrong), std::invalid_argument);
}

TEST_F(OptCLIKShould, checkBehaviorUpdateInitialJointPositionsFunction) {
    crf::utility::types::TaskVelocity zd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    crf::utility::types::JointPositions qAttr({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_NO_THROW(optCLIK_.reset(new OptCLIK(qInitial_, time_, configuration_, diagW_,
        objFunVector_, cardanTolerance_, K_, kinManip0_, alpha0_)));

    // With a wrong values in qInitial, the result is not successful
    crf::utility::types::TaskPose z(
        Eigen::Vector3d({-0.2919274767842110, 0.8697517351869412, 0.6748385376855995}),
        Eigen::Quaterniond(
            {0.8020650993106468, 0.3107536288616904, -0.3746593560288333, 0.3460550903500089}));
    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> resultCL1(  // NOLINT
        optCLIK_->getResults(qAttr, z, zd));
    // In the previous version, OptCLIK was using the distance measure below.
    // Because of that, results on a single iteration may differ from the ones from before.
    // The commented values are the esults from the previous version.
    // crf::utility::types::JointPositions qResult1(
    //     {0.1412341901917748, -0.6937242169865601, 2.1686510824362424,
    //     -0.6651824720994373, 0.1575640193040362, 0.2279567814299529});
    crf::utility::types::JointPositions qResult1(
        {0.14148716705663705, -0.59837426050420861, 1.9744942341218346,
        -0.4369716656104201, 0.1661464534035493, 0.09781174677134194});
    // crf::utility::types::JointPositions qdResult1(
    //     {-602.8259880368731274, -4777.6180239285467906, 9534.2584731854640268,
    //     -4634.9092994929333145, -521.1768424755659908, -169.2130318459826412});
    crf::utility::types::JointPositions qdResult1(
        {-601.56110371256182, -4300.8682415167896, 8563.4742316134252,
        -3493.8552670478475, -478.26467197800048, -819.93820513903734});
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_DOUBLE_EQ(std::get<0>(resultCL1)[i], qResult1[i]);
        ASSERT_NEAR(std::get<1>(resultCL1)[i], qdResult1[i], 1e-11);
    }
    ASSERT_EQ(std::get<3>(resultCL1),
        crf::control::inversekinematics::ResultFlags::endEffectorToleranceViolation);

    // After using updateInitialJointPositions() function with the qActual, which is the proper
    // qInitial
    JointPositions qActual({2.0, 3.5, 0.2, 1.8, 2.3, 0.7});
    ASSERT_NO_THROW(optCLIK_->updateInitialJointPositions(qActual));

    std::tuple<JointPositions, JointVelocities, JointAccelerations, crf::control::inversekinematics::ResultFlags> resultCL2(  // NOLINT
        optCLIK_->getResults(qAttr, z, zd));
    for (unsigned int i = 0; i < 6; i++) {
        ASSERT_DOUBLE_EQ(std::get<0>(resultCL2)[i], qActual[i]);
        ASSERT_NEAR(std::get<1>(resultCL2)[i], 0.0, 1e-11);
    }
    ASSERT_EQ(std::get<3>(resultCL2), crf::control::inversekinematics::ResultFlags::success);
}
