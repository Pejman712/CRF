/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/STI/ECE 2019
 * 
 *  ==================================================================================================
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"

using testing::_;
using testing::Return;

using crf::control::robotarmkinematics::RobotArmKDLKinematics;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;

class RobotArmKDLKinematicsShould : public ::testing::Test {
 protected:
    RobotArmKDLKinematicsShould(): logger_("RobotArmKDLKinematicsShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        config_ = std::make_shared<crf::actuators::robotarm::RobotArmConfiguration>();
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("cpproboticframework"));
        testDirName_ += "cpproboticframework/modules/Actuators/RobotArm/tests/config/";
    }
    ~RobotArmKDLKinematicsShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        std::ifstream robotData(testDirName_ + "goodConfiguration.json");
        nlohmann::json robotJSON = nlohmann::json::parse(robotData);
        ASSERT_TRUE(config_->parse(robotJSON));
        kinematics_ = std::make_unique<RobotArmKDLKinematics>(config_);
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> config_;
    std::unique_ptr<RobotArmKDLKinematics> kinematics_;
    std::string testDirName_;
};

TEST_F(RobotArmKDLKinematicsShould, getPositionForwardKinematicWrongNumberOfJointsTest) {
    JointPositions currentJointPositions(config_->getNumberOfJoints()-1);
    ASSERT_EQ(kinematics_->getPositionForwardKinematic(currentJointPositions), boost::none);

    currentJointPositions = JointPositions(config_->getNumberOfJoints()+1);
    ASSERT_EQ(kinematics_->getPositionForwardKinematic(currentJointPositions), boost::none);
}

TEST_F(RobotArmKDLKinematicsShould, getVelocityForwardKinematicWrongNumberOfJointsTest) {
    JointPositions currentJointPositions(config_->getNumberOfJoints()+1);
    JointVelocities currentJointVelocities(config_->getNumberOfJoints());
    ASSERT_EQ(kinematics_->getVelocityForwardKinematic(
        currentJointPositions, currentJointVelocities), boost::none);

    currentJointPositions = JointPositions(config_->getNumberOfJoints());
    currentJointVelocities = JointVelocities(config_->getNumberOfJoints()+1);
    ASSERT_EQ(kinematics_->getVelocityForwardKinematic(
        currentJointPositions, currentJointVelocities), boost::none);
}

TEST_F(RobotArmKDLKinematicsShould, getPositionInverseKinematicWrongNumberOfJointsTest) {
    JointPositions currentJointPositions(config_->getNumberOfJoints()+1);
    TaskPose frame;
    std::vector<JointPositions> empty;
    ASSERT_TRUE(kinematics_->getPositionInverseKinematic(frame, currentJointPositions).empty());
}

TEST_F(RobotArmKDLKinematicsShould, getVelocityInverseKinematicWrongNumberOfJointsTest) {
    JointPositions currentJointPositions(config_->getNumberOfJoints()+1);
    TaskVelocity taskVelocity;

    ASSERT_EQ(kinematics_->getVelocityInverseKinematic(taskVelocity, currentJointPositions),
        boost::none);
}

TEST_F(RobotArmKDLKinematicsShould, getPositionInverseKinematicOutOfReachabilityTest) {
    JointPositions currentJointPositions(config_->getNumberOfJoints());

    TaskPose frame(
        {1234.0, 0, 0},
        crf::math::rotation::CardanXYZ({0, 0, 0}));
    ASSERT_TRUE(kinematics_->getPositionInverseKinematic(frame, currentJointPositions).empty());
}

TEST_F(RobotArmKDLKinematicsShould,
    calculateGoodPositionInverseKinematicsFromObtainedUnchangedTaskPose) {
    JointPositions initialJointPositions({0, M_PI/2, M_PI/2, 0, M_PI/2, M_PI/2});
    auto initialTaskPose = kinematics_->getPositionForwardKinematic(
        initialJointPositions);
    ASSERT_TRUE(initialTaskPose);
    auto newJointPositions = kinematics_->getPositionInverseKinematic(
        *initialTaskPose, initialJointPositions);
    ASSERT_FALSE(newJointPositions.empty());

    bool oneElementMatchesOriginal = false;
    for (unsigned int i = 0; i < newJointPositions.size(); i++) {
        JointPositions dif(newJointPositions.at(i).raw() - initialJointPositions.raw());
        bool noDif = true;
        for (unsigned int j = 0; j < initialJointPositions.size(); j++) {
            if (fabs(dif[j]) > 0.01) {
                noDif = false;
            }
        }
        if (noDif) {
            oneElementMatchesOriginal = true;
            break;
        }
    }
    ASSERT_TRUE(oneElementMatchesOriginal);
}

TEST_F(RobotArmKDLKinematicsShould,
    DISABLED_calculateGoodPositionInverseKinematicsWithSmallChangeInObtainedTaskPose) {
    // in this position Kinova Arm has reasonable manipulability
    JointPositions initialJointPositions({0, M_PI, M_PI, 0, M_PI, M_PI});
    auto initialTaskPose = kinematics_->getPositionForwardKinematic(
        initialJointPositions);
    ASSERT_TRUE(initialTaskPose);
    TaskPose newTaskPose
        = multiply(TaskPose(
                {-0.05, -0.05, -0.05},
                Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0})),
            (*initialTaskPose));
    auto newJointPositions = kinematics_->getPositionInverseKinematic(
        newTaskPose, initialJointPositions);
    ASSERT_FALSE(newJointPositions.empty());
}

TEST_F(RobotArmKDLKinematicsShould,
    calculateGoodVelocityInverseKinematicsFromUnchangedVelocityForwardKinematics) {
    JointPositions initialJointPositions({0, M_PI/2, M_PI/2, 0, M_PI/2, M_PI/2});
    JointVelocities initialJointVelocities({0.01, 0.02, 0.03, 0.04, 0.05, 0.06});
    auto initialTaskVelocity = kinematics_->getVelocityForwardKinematic(
        initialJointPositions, initialJointVelocities);
    ASSERT_TRUE(initialTaskVelocity);
    auto newJointVelocities = kinematics_->getVelocityInverseKinematic(
        *initialTaskVelocity, initialJointPositions);
    ASSERT_TRUE(newJointVelocities);
    for (size_t i = 0; i < newJointVelocities->size(); i++) {
        ASSERT_FLOAT_EQ(initialJointVelocities[i], (*newJointVelocities)[i]);
    }
}

TEST_F(RobotArmKDLKinematicsShould,
    calculateGoodVelocityInverseKinematicsFromSlightlyChangedVelocityForwardKinematics) {
    JointPositions initialJointPositions({0, M_PI/2, M_PI/2, 0, M_PI/2, M_PI/2});
    JointVelocities initialJointVelocities({0.01, 0.02, 0.03, 0.04, 0.05, 0.06});
    auto initialTaskVelocity = kinematics_->getVelocityForwardKinematic(
        initialJointPositions, initialJointVelocities);
    ASSERT_TRUE(initialTaskVelocity);
    TaskVelocity newTaskVelocity(
        initialTaskVelocity.get().raw() + TaskVelocity({0.01, 0.01, 0.01, 0, 0, 0}).raw());
    auto newJointVelocities = kinematics_->getVelocityInverseKinematic(
        newTaskVelocity, initialJointPositions);
    ASSERT_TRUE(newJointVelocities);
}

TEST_F(RobotArmKDLKinematicsShould, calculateGoodManipulabilityAndSingularityTest) {
    // Positions based on KinovaJaco6DoFSpherical singularities
    JointPositions goodPosition({0, -0.6, 0.6, 0.0, -M_PI/2, 0});
    JointPositions zeroPosition({0, 0, 0, 0, 0, 0});
    JointPositions wristAlignmentSingularity({0, -0.3, -2.1, 0, 0, 0});
    JointPositions wristOverBaseSingularity({0, -M_PI, -2*M_PI/3, 0, 0, 0});

    auto manipulabilityGood = kinematics_->getManipulability(goodPosition);
    auto manipulabilityZero = kinematics_->getManipulability(zeroPosition);
    auto manipWristAlignmentSingularity = kinematics_->getManipulability(wristAlignmentSingularity);
    auto manipWristOverBaseSingularity = kinematics_->getManipulability(wristOverBaseSingularity);

    ASSERT_NE(manipulabilityGood, boost::none);
    ASSERT_NE(manipulabilityZero, boost::none);
    ASSERT_NE(manipWristAlignmentSingularity, boost::none);
    ASSERT_NE(manipWristOverBaseSingularity, boost::none);

    ASSERT_GT(manipulabilityGood.get(), manipulabilityZero.get());
    ASSERT_GT(manipulabilityGood.get(), manipWristAlignmentSingularity.get());
    ASSERT_GT(manipulabilityGood.get(), manipWristOverBaseSingularity.get());

    ASSERT_NEAR(0, manipulabilityZero.get(), 1e-12);
    ASSERT_NEAR(0, manipWristAlignmentSingularity.get(), 1e-8);
    ASSERT_NEAR(0, manipWristOverBaseSingularity.get(), 1e-12);
}

TEST_F(RobotArmKDLKinematicsShould, calculateGoodPoisitionForwardKinematicsWithLinearJoint) {
    // In this position Kinova Arm has reasonable manipulability
    std::ifstream robotData(testDirName_ + "goodConfigurationLinear.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_TRUE(config_->parse(robotJSON));
    kinematics_ = std::make_unique<RobotArmKDLKinematics>(config_);

    JointPositions initialJointsPos({0, 0.86, 0.36, 0, 1.16, 0.1});
    auto initialTaskPose = kinematics_->getPositionForwardKinematic(initialJointsPos);
    ASSERT_TRUE(initialTaskPose);
    ASSERT_NEAR(initialTaskPose.get().getPosition()(0), -0.822, 1e-3);
    ASSERT_NEAR(initialTaskPose.get().getPosition()(1), -0.009, 1e-3);
    ASSERT_NEAR(initialTaskPose.get().getPosition()(2), 0.783, 1e-3);
}
