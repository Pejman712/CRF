/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <regex>
#include <string>
#include <fstream>
#include <vector>
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "Robot/CombinedRobot/CombinedRobot.hpp"
#include "Robot/RobotMockConfiguration.hpp"
#include "Robot/IRobot.hpp"

using testing::_;
using testing::AnyNumber;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::SaveArg;
using testing::Return;

// using crf::actuators::robot::UniversalRobotRTDE;
// using crf::actuators::robot::UniversalRobotInterfaceMock;
using crf::actuators::robot::CombinedRobot;
using crf::actuators::robot::IRobot;
using crf::actuators::robot::RobotMockConfiguration;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::actuators::robot::CombinedRobotConfiguration;

class CombinedRobotShould: public ::testing::Test {
 protected:
    CombinedRobotShould():
        logger_("CombinedRobotShould") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("cpproboticframework"));
        configFilePaths_.push_back("cpproboticframework/modules/Actuators/Robot/tests/"
            "CombinedRobot/config/3DofRobot.json");
        configFilePaths_.push_back("cpproboticframework/modules/Actuators/Robot/tests/"
            "CombinedRobot/config/UR10eSimulation.json");
        std::ifstream configIfStream(testDirName + "cpproboticframework/modules/Actuators/Robot/"
            "tests/CombinedRobot/config/FccRobot.json");
        robotConfigFile_ = nlohmann::json::parse(configIfStream);
    }

    ~CombinedRobotShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        robot1_ = std::make_shared<RobotMockConfiguration>(3, configFilePaths_[0]);
        robot2_ = std::make_shared<RobotMockConfiguration>(6, configFilePaths_[1]);
        robot1_->getTaskObjectsImplemented = true;
        robot1_->configureMock();
        robot2_->configureMock();
        robots_.push_back(robot1_);
        robots_.push_back(robot2_);

        sut_ = std::make_unique<CombinedRobot>(
            robots_, CombinedRobotConfiguration(robotConfigFile_));
    }

    std::shared_ptr<RobotMockConfiguration> robot1_;
    std::shared_ptr<RobotMockConfiguration> robot2_;
    std::vector<std::shared_ptr<IRobot>> robots_;
    std::vector<std::string> configFilePaths_;
    nlohmann::json robotConfigFile_;
    crf::utility::logger::EventLogger logger_;

    std::unique_ptr<CombinedRobot> sut_;
};

TEST_F(CombinedRobotShould, returnTrueIfInitOnceAndDeInitOnce) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseIfDeinitializedMultipleTimes) {
    EXPECT_CALL(*robot1_, initialize()).Times(2);
    EXPECT_CALL(*robot1_, deinitialize()).Times(2);
    EXPECT_CALL(*robot2_, initialize()).Times(2);
    EXPECT_CALL(*robot2_, deinitialize()).Times(2);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseIfInitializedMultipleTimes) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnValidJointPositionsIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, getJointPositions()).Times(1);
    EXPECT_CALL(*robot2_, getJointPositions()).Times(1);
    JointPositions q(9);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions().value(), q));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseJointPositionsIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointPositions());
}

TEST_F(CombinedRobotShould, returnValidJointVelocitiesIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, getJointVelocities()).Times(1);
    EXPECT_CALL(*robot2_, getJointVelocities()).Times(1);
    JointVelocities qd(9);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointVelocities().value(), qd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseJointVelocitiesIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointVelocities());
}

TEST_F(CombinedRobotShould, returnFalseJointAccelerationsIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, getJointAccelerations()).Times(1);
    EXPECT_CALL(*robot2_, getJointAccelerations()).Times(1);
    JointForceTorques qdd(9);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointAccelerations().value(), qdd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseJointAccelerationsIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointAccelerations());
}

TEST_F(CombinedRobotShould, returnValidJointForceTorquesIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, getJointForceTorques()).Times(1);
    EXPECT_CALL(*robot2_, getJointForceTorques()).Times(1);
    JointForceTorques jT(9);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointForceTorques().value(), jT));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseJointForceTorquesIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointForceTorques());
}

TEST_F(CombinedRobotShould, returnFalseTaskPosesIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskPose());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseTaskPosesIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskPose());
}

TEST_F(CombinedRobotShould, returnFalseTaskVelocityIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseTaskVelocityIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskVelocity());
}

TEST_F(CombinedRobotShould, returnFalseTaskAccelerationIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskAcceleration());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseTaskAccelerationIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskAcceleration());
}

TEST_F(CombinedRobotShould, returnFalseTaskForceTorqueIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskForceTorque());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseTaskForceTorqueIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskForceTorque());
}

TEST_F(CombinedRobotShould, returnTrueIfFeasibleSetJointPositionsInput) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, setJointPositions(_, _, _, _)).Times(6);
    EXPECT_CALL(*robot2_, setJointPositions(_, _, _, _)).Times(6);

    std::vector<JointPositions> q {
        JointPositions({0, 0, 0, 0, 0, 0, 0, 0, 0}),
        JointPositions({1, 2, 3, 4, 5, 6, 7, 8, 9}),
        JointPositions({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6, 7.0, -8.1, 9.2}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointPositions(true, q[i]));
    }
    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointPositions(false, q[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTrueIfSetJointPositionsCorrectly) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, setJointPositions(_, _, _, _)).Times(6);
    EXPECT_CALL(*robot2_, setJointPositions(_, _, _, _)).Times(6);
    EXPECT_CALL(*robot1_, getJointPositions()).Times(2);
    EXPECT_CALL(*robot2_, getJointPositions()).Times(2);

    std::vector<JointPositions> q {
        JointPositions({0, 0, 0, 0, 0, 0, 0, 0, 0}),
        JointPositions({1, 2, 3, 4, 5, 6, 7, 8, 9}),
        JointPositions({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6, 7.0, -8.1, 9.2}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointPositions(true, q[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions().value(), q[q.size()-1]));

    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointPositions(false, q[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions().value(), q[q.size()-1]));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTrueIfSetJointVelocitiesSetCorrectly) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, setJointVelocities(_, _, _)).Times(6);
    EXPECT_CALL(*robot2_, setJointVelocities(_, _, _)).Times(6);
    EXPECT_CALL(*robot1_, getJointVelocities()).Times(2);
    EXPECT_CALL(*robot2_, getJointVelocities()).Times(2);

    std::vector<JointVelocities> q {
        JointVelocities({0, 0, 0, 0, 0, 0, 0, 0, 0}),
        JointVelocities({1, 2, 3, 4, 5, 6, 7, 8, 9}),
        JointVelocities({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6, 7.0, -8.1, 9.2}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointVelocities(true, q[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getJointVelocities().value(), q[q.size()-1]));

    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointVelocities(false, q[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getJointVelocities().value(), q[q.size()-1]));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTrueIfSetJointForceTorquesSetCorrectly) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, setJointForceTorques(_, _)).Times(6);
    EXPECT_CALL(*robot2_, setJointForceTorques(_, _)).Times(6);
    EXPECT_CALL(*robot1_, getJointForceTorques()).Times(2);
    EXPECT_CALL(*robot2_, getJointForceTorques()).Times(2);

    std::vector<JointForceTorques> t {
        JointForceTorques({0, 0, 0, 0, 0, 0, 0, 0, 0}),
        JointForceTorques({1, 2, 3, 4, 5, 6, 7, 8, 9}),
        JointForceTorques({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6, 7.0, -8.1, 9.2}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < t.size(); i++) {
        ASSERT_TRUE(sut_->setJointForceTorques(true, t[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getJointForceTorques().value(), t[t.size()-1]));

    for (int i = 0; i < t.size(); i++) {
        ASSERT_TRUE(sut_->setJointForceTorques(false, t[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getJointForceTorques().value(), t[t.size()-1]));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalsesetTaskPoseIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    TaskPose dummy(Eigen::Vector3d({1, 0, 0}),
                   crf::math::rotation::CardanXYZ({0, 0, 0}));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setTaskPose(true, dummy));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalsesetTaskVelocityIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    TaskVelocity dummy {1, 0, 0, 0, 0, 0};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setTaskVelocity(true, dummy));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalsesetTaskForceTorqueIfInitialized) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    TaskForceTorque dummy {1, 0, 0, 0, 0, 0};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setTaskForceTorque(true, dummy));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTrueIfSetProfileJointVelocitiesCorrectly) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, setProfileJointVelocities(_)).Times(6);
    EXPECT_CALL(*robot2_, setProfileJointVelocities(_)).Times(6);
    EXPECT_CALL(*robot1_, getProfileJointVelocities()).Times(2);
    EXPECT_CALL(*robot2_, getProfileJointVelocities()).Times(2);

    std::vector<JointVelocities> qd {
        JointVelocities({0, 0, 0, 0, 0, 0, 0, 0, 0}),
        JointVelocities({1, 2, 3, 4, 5, 6, 7, 8, 9}),
        JointVelocities({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6, 7.0, -8.1, 9.2}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qd.size(); i++) {
        ASSERT_TRUE(sut_->setProfileJointVelocities(qd[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getProfileJointVelocities().value(), qd[qd.size()-1]));

    for (int i = 0; i < qd.size(); i++) {
        ASSERT_TRUE(sut_->setProfileJointVelocities(qd[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getProfileJointVelocities().value(), qd[qd.size()-1]));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTrueIfSetProfileJointAccelerationsCorrectly) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, setProfileJointAccelerations(_)).Times(6);
    EXPECT_CALL(*robot2_, setProfileJointAccelerations(_)).Times(6);
    EXPECT_CALL(*robot1_, getProfileJointAccelerations()).Times(2);
    EXPECT_CALL(*robot2_, getProfileJointAccelerations()).Times(2);

    std::vector<JointAccelerations> qdd {
        JointAccelerations({0, 0, 0, 0, 0, 0, 0, 0, 0}),
        JointAccelerations({1, 2, 3, 4, 5, 6, 7, 8, 9}),
        JointAccelerations({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6, 7.0, -8.1, 9.2}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qdd.size(); i++) {
        ASSERT_TRUE(sut_->setProfileJointAccelerations(qdd[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getProfileJointAccelerations().value(), qdd[qdd.size()-1]));

    for (int i = 0; i < qdd.size(); i++) {
        ASSERT_TRUE(sut_->setProfileJointAccelerations(qdd[i]));
    }

    ASSERT_TRUE(areAlmostEqual(sut_->getProfileJointAccelerations().value(), qdd[qdd.size()-1]));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTrueIfFeasibleGravityVector) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, setGravity(_)).Times(4);
    EXPECT_CALL(*robot2_, setGravity(_)).Times(4);
    EXPECT_CALL(*robot1_, getTaskPose()).Times(4);

    std::array<double, 3> g1{9.81, 0, 0};
    std::array<double, 3> g2{0, 9.81, 0};
    std::array<double, 3> g3{0, 0, 9.81};
    std::array<double, 3> g4{9.81/sqrt(3), 9.81/sqrt(3), 9.81/sqrt(3)};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setGravity(g1).value());
    ASSERT_TRUE(sut_->setGravity(g2).value());
    ASSERT_TRUE(sut_->setGravity(g3).value());
    ASSERT_TRUE(sut_->setGravity(g4).value());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnFalseIfBadGravityVector) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);

    std::array<double, 3> g1{0, 0, 1};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setGravity(g1));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTureIfSoftStopped) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, softStop()).Times(1);
    EXPECT_CALL(*robot2_, softStop()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->softStop());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTureIfHardStopped) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, hardStop()).Times(1);
    EXPECT_CALL(*robot2_, hardStop()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->hardStop());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CombinedRobotShould, returnTureIfSetGetBrakesCorrectly) {
    EXPECT_CALL(*robot1_, initialize()).Times(1);
    EXPECT_CALL(*robot1_, deinitialize()).Times(1);
    EXPECT_CALL(*robot2_, initialize()).Times(1);
    EXPECT_CALL(*robot2_, deinitialize()).Times(1);
    EXPECT_CALL(*robot1_, setBrakes(_)).Times(3);
    EXPECT_CALL(*robot2_, setBrakes(_)).Times(3);
    EXPECT_CALL(*robot1_, getBrakes()).Times(1);
    EXPECT_CALL(*robot2_, getBrakes()).Times(1);

    std::vector<std::vector<bool>> dummy {
        {1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 0, 1, 0, 1, 1, 0, 0, 0}};

    ASSERT_TRUE(sut_->initialize());

    for (int i=0; i < dummy.size(); i++) {
        ASSERT_TRUE(sut_->setBrakes(dummy[i]));
    }

    ASSERT_EQ(sut_->getBrakes().value(), dummy[dummy.size()-1]);

    ASSERT_TRUE(sut_->deinitialize());
}

