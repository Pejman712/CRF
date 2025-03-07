/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
*/

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "ForwardKinematics/KinChainForwardKinematics/KinChainForwardKinematics.hpp"
#include "KinematicChain/URDFKinematicChain/URDFKinematicChain.hpp"

using crf::control::forwardkinematics::IForwardKinematics;
using crf::control::forwardkinematics::KinChainForwardKinematics;

using crf::math::kinematicchain::IKinematicChain;
using crf::math::kinematicchain::URDFKinematicChain;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;

class KinChainForwardKinematicsShould : public ::testing::Test {
 protected:
    KinChainForwardKinematicsShould():
        logger_("KinChainForwardKinematicsShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("ForwardKinematics"));
        testDirName += "ForwardKinematics/tests/config/";

        std::string pathToURDF =
            testDirName + "KinematicChainCombinedTestingBasedOnSPSReducedTaskSpace.urdf";
        std::string toolName = "kinova_end_effector";

        kinChain_.reset(new URDFKinematicChain(pathToURDF, toolName, ""));
    }
    ~KinChainForwardKinematicsShould() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<IKinematicChain> kinChain_;  // TODO(any) use mock
    std::unique_ptr<IForwardKinematics> forwardKinematics_;

    const double maxAbsError_ = 1e-12;
};

TEST_F(KinChainForwardKinematicsShould, returnFalseIfInputDimensionQIsDifferentThanTheExpected) {
    ASSERT_NO_THROW(forwardKinematics_.reset(new KinChainForwardKinematics(kinChain_)));
    JointPositions q = JointPositions({0.0, 0.0, 0.3, 0.0, 0.0});
    // Wrong q
    std::optional<TaskPose> z = forwardKinematics_->getPose(q);
    ASSERT_FALSE(z);
    ASSERT_EQ(z, std::nullopt);
}

TEST_F(KinChainForwardKinematicsShould, returnNullOptIfNotImplemented) {
    ASSERT_NO_THROW(forwardKinematics_.reset(new KinChainForwardKinematics(kinChain_)));

    JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0});
    JointVelocities qd = JointVelocities({0.0, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.2, 0.0});
    JointAccelerations qdd = JointAccelerations({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_EQ(forwardKinematics_->getVelocity(q, qd), std::nullopt);
    ASSERT_EQ(forwardKinematics_->getAcceleration(q, qd, qdd), std::nullopt);
}

TEST_F(KinChainForwardKinematicsShould, returnCorrectResults) {
    JointPositions q = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_NO_THROW(forwardKinematics_.reset(new KinChainForwardKinematics(kinChain_)));
    std::optional<TaskPose> z = forwardKinematics_->getPose(q);
    ASSERT_TRUE(z);
    ASSERT_NEAR(
        z.value().getPosition()(0),
        0.21991540529583217,
        maxAbsError_);
    ASSERT_NEAR(
        z.value().getPosition()(1),
        -1.336169030869699,
        maxAbsError_);
    ASSERT_NEAR(
        z.value().getPosition()(2),
        -0.53923816280533932,
        maxAbsError_);
    ASSERT_NEAR(
        z.value().getQuaternion().w(),
        0.53296276972012624,
        maxAbsError_);
    ASSERT_NEAR(
        z.value().getQuaternion().x(),
        -0.13883757523469051,
        maxAbsError_);
    ASSERT_NEAR(
        z.value().getQuaternion().y(),
        -0.089125394437540126,
        maxAbsError_);
    ASSERT_NEAR(
        z.value().getQuaternion().z(),
        0.82989847443019005,
        maxAbsError_);

    JointPositions q2 = JointPositions({0.0, 0.0, 0.0, 0.0, 1.6, -0.7, 0.3, 0.2, 0.15, 0.4});

    ASSERT_NO_THROW(forwardKinematics_.reset(new KinChainForwardKinematics(kinChain_)));
    std::optional<TaskPose> z2 = forwardKinematics_->getPose(q2);
    ASSERT_TRUE(z2);
    ASSERT_NEAR(
        z2.value().getPosition()(0),
        -0.25821541772541234,
        maxAbsError_);
    ASSERT_NEAR(
        z2.value().getPosition()(1),
        -1.4841561230128562,
        maxAbsError_);
    ASSERT_NEAR(
        z2.value().getPosition()(2),
        0.31045313344562309,
        maxAbsError_);
    ASSERT_NEAR(
        z2.value().getQuaternion().w(),
        0.088168070299989854,
        maxAbsError_);
    ASSERT_NEAR(
        z2.value().getQuaternion().x(),
        0.71777613363596049,
        maxAbsError_);
    ASSERT_NEAR(
        z2.value().getQuaternion().y(),
        0.58380486822443256,
        maxAbsError_);
    ASSERT_NEAR(
        z2.value().getQuaternion().z(),
        0.36904700134216117,
        maxAbsError_);
}
