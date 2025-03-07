/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 * 
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"

using crf::control::forwardkinematics::IForwardKinematics;
using crf::control::forwardkinematics::MathExprForwardKinematics;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;

class MathExprForwardKinematicsShould: public ::testing::Test {
 protected:
    MathExprForwardKinematicsShould(): logger_("MathExprForwardKinematicsShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("ForwardKinematics"));
        testDirName_ += "ForwardKinematics/tests/config/";
    }
    ~MathExprForwardKinematicsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        std::ifstream jsonConfigPath(testDirName_ + "UR10ForwardKinematicsTests.json");
        jsonConfig_ = nlohmann::json::parse(jsonConfigPath);
        lx_ = {0.0, 0.0, 0.6127, 0.57155, 0.0, 0.0};
        ly_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.11655};
        lz_ = {0.0, 0.1807, 0.0, 0.0, 0.17415, 0.11985};
    }

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    nlohmann::json jsonConfig_;
    std::vector<double> lx_, ly_, lz_;
    std::unique_ptr<IForwardKinematics> forwardKinematics_;
};

TEST_F(MathExprForwardKinematicsShould, ThrowEmptyJSONExpression) {
    std::ifstream jsonConfigPathV2(
        testDirName_ + "UR10ForwardKinematicsTestsV2EmptyExpression.json");
    nlohmann::json jsonConfigV2 = nlohmann::json::parse(jsonConfigPathV2);

    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV2, lx_, ly_, lz_)), std::runtime_error);
}

TEST_F(MathExprForwardKinematicsShould, ThrowVariablesAreNotCorrectlyInitialized) {
    std::vector<double> lz = {0.0, 0.181, 0.613, 0.571, 0.120};

    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfig_, lx_, ly_, lz)), std::runtime_error);
}

TEST_F(MathExprForwardKinematicsShould, ThrowJSONExpressionError) {
    std::ifstream jsonConfigPathV3(
        testDirName_ + "UR10ForwardKinematicsTestsV3ExpressionError.json");
    nlohmann::json jsonConfigV3 = nlohmann::json::parse(jsonConfigPathV3);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV3, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV7(
        testDirName_ + "UR10ForwardKinematicsTestsV7ExpressionError.json");
    nlohmann::json jsonConfigV7 = nlohmann::json::parse(jsonConfigPathV7);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV7, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV8(
        testDirName_ + "UR10ForwardKinematicsTestsV8ExpressionError.json");
    nlohmann::json jsonConfigV8 = nlohmann::json::parse(jsonConfigPathV8);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV8, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV4(
        testDirName_ + "UR10ForwardKinematicsTestsV4ExpressionError.json");
    nlohmann::json jsonConfigV4 = nlohmann::json::parse(jsonConfigPathV4);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV4, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV9(
        testDirName_ + "UR10ForwardKinematicsTestsV9ExpressionError.json");
    nlohmann::json jsonConfigV9 = nlohmann::json::parse(jsonConfigPathV9);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV9, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV10(
        testDirName_ + "UR10ForwardKinematicsTestsV10ExpressionError.json");
    nlohmann::json jsonConfigV10 = nlohmann::json::parse(jsonConfigPathV10);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV10, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV11(
        testDirName_ + "UR10ForwardKinematicsTestsV11ExpressionError.json");
    nlohmann::json jsonConfigV11 = nlohmann::json::parse(jsonConfigPathV11);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV11, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV12(
        testDirName_ + "UR10ForwardKinematicsTestsV12ExpressionError.json");
    nlohmann::json jsonConfigV12 = nlohmann::json::parse(jsonConfigPathV12);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV12, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV13(
        testDirName_ + "UR10ForwardKinematicsTestsV13ExpressionError.json");
    nlohmann::json jsonConfigV13 = nlohmann::json::parse(jsonConfigPathV13);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV13, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV14(
        testDirName_ + "UR10ForwardKinematicsTestsV14ExpressionError.json");
    nlohmann::json jsonConfigV14 = nlohmann::json::parse(jsonConfigPathV14);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV14, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV15(
        testDirName_ + "UR10ForwardKinematicsTestsV15ExpressionError.json");
    nlohmann::json jsonConfigV15 = nlohmann::json::parse(jsonConfigPathV15);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV15, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV16(
        testDirName_ + "UR10ForwardKinematicsTestsV16ExpressionError.json");
    nlohmann::json jsonConfigV16 = nlohmann::json::parse(jsonConfigPathV16);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV16, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV17(
        testDirName_ + "UR10ForwardKinematicsTestsV17ExpressionError.json");
    nlohmann::json jsonConfigV17 = nlohmann::json::parse(jsonConfigPathV17);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV17, lx_, ly_, lz_)), std::runtime_error);

    std::ifstream jsonConfigPathV18(
        testDirName_ + "UR10ForwardKinematicsTestsV18ExpressionError.json");
    nlohmann::json jsonConfigV18 = nlohmann::json::parse(jsonConfigPathV18);
    ASSERT_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfigV18, lx_, ly_, lz_)), std::runtime_error);
}

TEST_F(MathExprForwardKinematicsShould, returnFalseIfJSONFileIsIncomplete) {
    std::ifstream jsonConfigPathV5(
        testDirName_ + "UR10ForwardKinematicsTestsV5IncompleteFile.json");
    nlohmann::json jsonConfigV5 = nlohmann::json::parse(jsonConfigPathV5);

    ASSERT_NO_THROW(forwardKinematics_.reset(
        new MathExprForwardKinematics(jsonConfigV5, lx_, ly_, lz_)));

    JointPositions q = JointPositions({0.0, 0.0, 0.3, 0.0, 0.0, 0.0});
    std::optional<TaskPose> z = forwardKinematics_->getPose(q);
    ASSERT_FALSE(z);
    ASSERT_EQ(z, std::nullopt);

    JointVelocities qd = JointVelocities({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::optional<TaskVelocity> zd = forwardKinematics_->getVelocity(q, qd);
    ASSERT_FALSE(zd);
    ASSERT_EQ(zd, std::nullopt);

    JointAccelerations qdd = JointAccelerations({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::optional<TaskAcceleration> zdd = forwardKinematics_->getAcceleration(q, qd, qdd);
    ASSERT_FALSE(zdd);
    ASSERT_EQ(zdd, std::nullopt);
}

TEST_F(MathExprForwardKinematicsShould, returnFalseIfInputDimensionQIsDifferentThanTheExpected) {
    ASSERT_NO_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfig_, lx_, ly_, lz_)));
    JointPositions q = JointPositions({0.0, 0.0, 0.3, 0.0, 0.0});
    // Wrong q
    std::optional<TaskPose> z = forwardKinematics_->getPose(q);
    ASSERT_FALSE(z);
    ASSERT_EQ(z, std::nullopt);

    JointVelocities qd = JointVelocities({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // Wrong q, Good qd
    std::optional<TaskVelocity> zd = forwardKinematics_->getVelocity(q, qd);
    ASSERT_FALSE(zd);
    ASSERT_EQ(zd, std::nullopt);

    JointAccelerations qdd = JointAccelerations({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // Wrong q, Good qd, Good qdd
    std::optional<TaskAcceleration> zdd = forwardKinematics_->getAcceleration(q, qd, qdd);
    ASSERT_FALSE(zdd);
    ASSERT_EQ(zdd, std::nullopt);

    q = JointPositions({0.0, 0.0, 0.3, 0.0, 0.0, 0.0});
    qd = JointVelocities({0.0, 0.0, 0.0, 0.0, 0.0});
    // Good q, Wrong qd
    zd = forwardKinematics_->getVelocity(q, qd);
    ASSERT_FALSE(zd);
    ASSERT_EQ(zd, std::nullopt);

    // Good q, Wrong qd, Good qdd
    zdd = forwardKinematics_->getAcceleration(q, qd, qdd);
    ASSERT_FALSE(zdd);
    ASSERT_EQ(zdd, std::nullopt);

    qd = JointVelocities({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    qdd = JointAccelerations({0.0, 0.0, 0.0, 0.0, 0.0});
    // Wrong q, Good qd, Wrong qdd
    zdd = forwardKinematics_->getAcceleration(q, qd, qdd);
    ASSERT_FALSE(zdd);
    ASSERT_EQ(zdd, std::nullopt);
}

TEST_F(MathExprForwardKinematicsShould, returnCorrectResults) {
    JointPositions q = JointPositions({0.0, 0.0, 0.3, 0.0, 0.0, 0.0});
    JointVelocities qd = JointVelocities({0.6, 0.0, 0.0, 0.0, 0.2, 0.0});
    JointAccelerations qdd = JointAccelerations({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_NO_THROW(forwardKinematics_.reset(
        new MathExprForwardKinematics(jsonConfig_, lx_, ly_, lz_)));
    std::optional<TaskPose> z = forwardKinematics_->getPose(q);
    ASSERT_TRUE(z);
    ASSERT_DOUBLE_EQ(z.value().getPosition()(0),
        -1.1233044735913786);
    ASSERT_DOUBLE_EQ(z.value().getPosition()(1),
        -0.2907);
    ASSERT_DOUBLE_EQ(z.value().getPosition()(2),
        -0.1027016523389925);
    ASSERT_DOUBLE_EQ(z.value().getQuaternion().w(),
        0.6991667342497078);
    ASSERT_DOUBLE_EQ(z.value().getQuaternion().x(),
        0.6991667342497078);
    ASSERT_DOUBLE_EQ(z.value().getQuaternion().y(),
        -0.10566871683993562);
    ASSERT_DOUBLE_EQ(z.value().getQuaternion().z(),
        0.10566871683993562);
    std::optional<TaskVelocity> zd = forwardKinematics_->getVelocity(q, qd);
    ASSERT_TRUE(zd);
    ASSERT_DOUBLE_EQ(zd.value()[0], 0.1521511064384821);
    ASSERT_DOUBLE_EQ(zd.value()[1], -0.6739826841548272);
    ASSERT_DOUBLE_EQ(zd.value()[2], -0.0068885760172758253);
    ASSERT_DOUBLE_EQ(zd.value()[3], 0.0591040413322679);
    ASSERT_DOUBLE_EQ(zd.value()[4], 0.0);
    ASSERT_DOUBLE_EQ(zd.value()[5], 0.4089327021748788);
    std::optional<TaskAcceleration> zdd = forwardKinematics_->getAcceleration(q, qd, qdd);
    ASSERT_TRUE(zdd);
    ASSERT_DOUBLE_EQ(zdd.value()[0], 0.4043896104928962);
    ASSERT_DOUBLE_EQ(zdd.value()[1], 0.0825913277261786);
    ASSERT_DOUBLE_EQ(zdd.value()[2], 0.0);
    ASSERT_DOUBLE_EQ(zdd.value()[3], 0.0);
    ASSERT_DOUBLE_EQ(zdd.value()[4], 0.035462424799360744);
    ASSERT_DOUBLE_EQ(zdd.value()[5], 0.0);

    std::ifstream jsonConfigPathV6(testDirName_ + "UR10ForwardKinematicsTestsV6Quaternion.json");
    nlohmann::json jsonConfigV6 = nlohmann::json::parse(jsonConfigPathV6);
    ASSERT_NO_THROW(forwardKinematics_.reset(
        new MathExprForwardKinematics(jsonConfigV6, lx_, ly_, lz_)));
    std::optional<TaskPose> z2 = forwardKinematics_->getPose(q);
    ASSERT_TRUE(z2);
    ASSERT_DOUBLE_EQ(z2.value().getPosition()(0),
        -1.1233044735913786);
    ASSERT_DOUBLE_EQ(z2.value().getPosition()(1),
        -0.2907);
    ASSERT_DOUBLE_EQ(z2.value().getPosition()(2),
        -0.1027016523389925);
    ASSERT_DOUBLE_EQ(z2.value().getQuaternion().w(),
        0.6991667342497078);
    ASSERT_DOUBLE_EQ(z2.value().getQuaternion().x(),
        0.6991667342497078);
    ASSERT_DOUBLE_EQ(z2.value().getQuaternion().y(),
        -0.10566871683993562);
    ASSERT_DOUBLE_EQ(z2.value().getQuaternion().z(),
        0.10566871683993562);
    std::optional<TaskVelocity> zd2 = forwardKinematics_->getVelocity(q, qd);
    ASSERT_TRUE(zd2);
    ASSERT_DOUBLE_EQ(zd2.value()[0], 0.1521511064384821);
    ASSERT_DOUBLE_EQ(zd2.value()[1], -0.6739826841548272);
    ASSERT_DOUBLE_EQ(zd2.value()[2], -0.0068885760172758253);
    ASSERT_DOUBLE_EQ(zd2.value()[3], 0.0591040413322679);
    ASSERT_DOUBLE_EQ(zd2.value()[4], 0.0);
    ASSERT_DOUBLE_EQ(zd2.value()[5], 0.4089327021748788);
    std::optional<TaskAcceleration> zdd2 = forwardKinematics_->getAcceleration(q, qd, qdd);
    ASSERT_TRUE(zdd2);
    ASSERT_DOUBLE_EQ(zdd2.value()[0], 0.4043896104928962);
    ASSERT_DOUBLE_EQ(zdd2.value()[1], 0.0825913277261786);
    ASSERT_DOUBLE_EQ(zdd2.value()[2], 0.0);
    ASSERT_DOUBLE_EQ(zdd2.value()[3], 0.0);
    ASSERT_DOUBLE_EQ(zdd2.value()[4], 0.035462424799360744);
    ASSERT_DOUBLE_EQ(zdd2.value()[5], 0.0);
}

TEST_F(MathExprForwardKinematicsShould, SeveralConstructorsTest) {
    JointPositions q = JointPositions({M_PI/2.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    JointVelocities qd = JointVelocities({0.6, 0.0, 0.0, 0.0, 0.2, 0.0});
    JointAccelerations qdd = JointAccelerations({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::ifstream jsonConfigPathV6(testDirName_ + "UR10ForwardKinematicsTestsV6Quaternion.json");
    nlohmann::json jsonConfigV6 = nlohmann::json::parse(jsonConfigPathV6);

    ASSERT_NO_THROW(forwardKinematics_.reset(
        new MathExprForwardKinematics(jsonConfig_, lx_, ly_, lz_)));
    std::optional<TaskPose> z = forwardKinematics_->getPose(q);
    ASSERT_TRUE(z);

    ASSERT_NO_THROW(forwardKinematics_.reset(
        new MathExprForwardKinematics(jsonConfigV6, lx_, ly_, lz_)));
    std::optional<TaskPose> z2 = forwardKinematics_->getPose(q);
    ASSERT_TRUE(z2);

    ASSERT_NO_THROW(forwardKinematics_.reset(
        new MathExprForwardKinematics(jsonConfig_, lx_, ly_, lz_)));
    std::optional<TaskPose> z3 = forwardKinematics_->getPose(q);
    ASSERT_TRUE(z3);

    ASSERT_NO_THROW(forwardKinematics_.reset(
        new MathExprForwardKinematics(jsonConfigV6, lx_, ly_, lz_)));
    std::optional<TaskPose> z4 = forwardKinematics_->getPose(q);
    ASSERT_TRUE(z4);

    ASSERT_TRUE(areAlmostEqual(z.value(), z2.value()));
    ASSERT_TRUE(areAlmostEqual(z.value(), z3.value()));
    ASSERT_TRUE(areAlmostEqual(z.value(), z4.value()));
}

TEST_F(MathExprForwardKinematicsShould, SeveralFunctionCallsChangingInputTest) {
    ASSERT_NO_THROW(forwardKinematics_.reset(new MathExprForwardKinematics(
        jsonConfig_, lx_, ly_, lz_)));
    JointPositions q = JointPositions({M_PI/2.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::optional<TaskPose> z = forwardKinematics_->getPose(q);

    JointPositions q2 = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::optional<TaskPose> z2 = forwardKinematics_->getPose(q2);

    ASSERT_TRUE(!areAlmostEqual(z.value(), z2.value()));

    JointPositions q3 = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    for (unsigned int i = 0; i < 3; i++) {
        std::optional<TaskPose> z3 = forwardKinematics_->getPose(q3);
        switch (i) {
            case 0: ASSERT_TRUE(areAlmostEqual(z2.value(), z3.value())); break;
            case 1: ASSERT_TRUE(areAlmostEqual(z.value(), z3.value())); break;
            case 2:
                ASSERT_TRUE(!areAlmostEqual(z.value(), z3.value()));
                ASSERT_TRUE(!areAlmostEqual(z2.value(), z3.value()));
                break;
        }
        q3 = JointPositions(q3.raw() + JointPositions({M_PI/2.0, 0.0, 0.0, 0.0, 0.0, 0.0}).raw());
    }
}

TEST_F(MathExprForwardKinematicsShould, reducedFKTest) {
    std::ifstream jsonConfigPath(
        testDirName_ + "UR10ForwardKinematicsTestsReduced.json");
    nlohmann::json jsonConfig = nlohmann::json::parse(jsonConfigPath);
    std::vector<double> lx = {0.6127, 0.57155, 0.0, 0.0};
    std::vector<double> ly = {0.0, 0.0, 0.0, 0.11655};
    std::vector<double> lz = {0.1807, 0.0, 0.0, 0.17415};

    ASSERT_NO_THROW(forwardKinematics_.reset(
        new MathExprForwardKinematics(jsonConfig, lx, ly, lz)));

    JointPositions q = JointPositions({0.3, 0.0, 0.0, 0.0});
    JointVelocities qd = JointVelocities({0.6, 0.0, 0.2, 0.0});
    JointAccelerations qdd = JointAccelerations({0.0, 0.0, 0.0, 0.0});

    std::optional<TaskPose> z = forwardKinematics_->getPose(q);
    ASSERT_TRUE(z);
    ASSERT_DOUBLE_EQ(z.value().getPosition()(0), cos(q[0]) + lx[0]);
    ASSERT_DOUBLE_EQ(z.value().getPosition()(1), cos(q[1]) + ly[0]);
    ASSERT_TRUE(std::isnan(z.value().getPosition()(2)));
    ASSERT_TRUE(std::isnan(z.value().getCardanXYZ()[0]));
    ASSERT_TRUE(std::isnan(z.value().getCardanXYZ()[1]));
    ASSERT_DOUBLE_EQ(z.value().getCardanXYZ()[2], cos(q[3]) + lx[1]);
    std::optional<TaskVelocity> zd = forwardKinematics_->getVelocity(q, qd);
    ASSERT_TRUE(zd);
    ASSERT_DOUBLE_EQ(zd.value()[0], cos(qd[0]) + sin(q[0]) + ly[1]);
    ASSERT_DOUBLE_EQ(zd.value()[1], cos(qd[1]) + sin(q[1]) + lz[1]);
    ASSERT_TRUE(std::isnan(zd.value()[2]));
    ASSERT_TRUE(std::isnan(zd.value()[3]));
    ASSERT_TRUE(std::isnan(zd.value()[4]));
    ASSERT_DOUBLE_EQ(zd.value()[5], cos(qd[3]) + sin(q[3]) + ly[2]);
    std::optional<TaskAcceleration> zdd = forwardKinematics_->getAcceleration(q, qd, qdd);
    ASSERT_TRUE(zdd);
    ASSERT_DOUBLE_EQ(zdd.value()[0], sin(qdd[0]) + cos(qd[0]) + sin(q[0]) + lz[2]);
    ASSERT_DOUBLE_EQ(zdd.value()[1], sin(qdd[1]) + cos(qd[1]) + sin(q[1]) + lx[3]);
    ASSERT_TRUE(std::isnan(zdd.value()[2]));
    ASSERT_TRUE(std::isnan(zdd.value()[3]));
    ASSERT_TRUE(std::isnan(zdd.value()[4]));
    ASSERT_DOUBLE_EQ(zdd.value()[5], sin(qdd[3]) + cos(qd[3]) + sin(q[3]) + lz[3]);
}
