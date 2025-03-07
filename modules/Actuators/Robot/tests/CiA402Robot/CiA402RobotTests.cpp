/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playan Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <memory>
#include <regex>
#include <string>
#include <fstream>
#include <vector>

#include <boost/optional.hpp>
#include <nlohmann/json.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Robot/CiA402Robot/CiA402Robot.hpp"
#include "CANopenDrivers/CiA402/CiA402DriverMockConfiguration.hpp"

#include "Types/Types.hpp"
#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::AnyNumber;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::SaveArg;
using testing::Return;

using crf::actuators::robot::CiA402Robot;
using crf::actuators::robot::CiA402RobotConfiguration;
using crf::actuators::robot::IRobot;

using crf::devices::canopendrivers::CiA402DriverMockConfiguration;

using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

class CiA402RobotShould: public ::testing::Test {
 protected:
    CiA402RobotShould():
        logger_("CiA402RobotShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~CiA402RobotShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("cpproboticframework"));
        testDirName += "cpproboticframework/modules/Actuators/Robot/tests/CiA402Robot/config/PRBT6.json";  // NOLINT
        nlohmann::json config = nlohmann::json::parse(std::ifstream(testDirName));

        motorMock1_ = std::make_shared<CiA402DriverMockConfiguration>();
        motorMock1_->configureMock();
        motorMock2_ = std::make_shared<CiA402DriverMockConfiguration>();
        motorMock2_->configureMock();
        motorMock3_ = std::make_shared<CiA402DriverMockConfiguration>();
        motorMock3_->configureMock();
        motorMock4_ = std::make_shared<CiA402DriverMockConfiguration>();
        motorMock4_->configureMock();
        motorMock5_ = std::make_shared<CiA402DriverMockConfiguration>();
        motorMock5_->configureMock();
        motorMock6_ = std::make_shared<CiA402DriverMockConfiguration>();
        motorMock6_->configureMock();

        motors_.push_back(motorMock1_);
        motors_.push_back(motorMock2_);
        motors_.push_back(motorMock3_);
        motors_.push_back(motorMock4_);
        motors_.push_back(motorMock5_);
        motors_.push_back(motorMock6_);

        sut_ = std::make_unique<CiA402Robot>(motors_, CiA402RobotConfiguration(config));
    }

    std::shared_ptr<CiA402DriverMockConfiguration> motorMock1_;
    std::shared_ptr<CiA402DriverMockConfiguration> motorMock2_;
    std::shared_ptr<CiA402DriverMockConfiguration> motorMock3_;
    std::shared_ptr<CiA402DriverMockConfiguration> motorMock4_;
    std::shared_ptr<CiA402DriverMockConfiguration> motorMock5_;
    std::shared_ptr<CiA402DriverMockConfiguration> motorMock6_;
    std::vector<std::shared_ptr<ICiA402Driver>> motors_;

    std::unique_ptr<CiA402Robot> sut_;

    crf::utility::logger::EventLogger logger_;
};

TEST_F(CiA402RobotShould, returnTrueIfInitOnceAndDeInitOnce) {
    EXPECT_CALL(*motorMock1_, initialize()).Times(1);
    EXPECT_CALL(*motorMock2_, initialize()).Times(1);
    EXPECT_CALL(*motorMock3_, initialize()).Times(1);
    EXPECT_CALL(*motorMock4_, initialize()).Times(1);
    EXPECT_CALL(*motorMock5_, initialize()).Times(1);
    EXPECT_CALL(*motorMock6_, initialize()).Times(1);

    EXPECT_CALL(*motorMock1_, deinitialize()).Times(1);
    EXPECT_CALL(*motorMock2_, deinitialize()).Times(1);
    EXPECT_CALL(*motorMock3_, deinitialize()).Times(1);
    EXPECT_CALL(*motorMock4_, deinitialize()).Times(1);
    EXPECT_CALL(*motorMock5_, deinitialize()).Times(1);
    EXPECT_CALL(*motorMock6_, deinitialize()).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnValidJointPositionssIfInitialized) {
    JointPositions q(6);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions().value(), q));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalseJointPositionsIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointPositions());
}

TEST_F(CiA402RobotShould, returnValidJointsVelocitiesIfInitialized) {
    JointVelocities qd(6);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointVelocities().value(), qd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalseJointsVelocitiesIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointVelocities());
}

TEST_F(CiA402RobotShould, returnFalseJointAccelerationsIfInitialized) {
    JointForceTorques qdd(6);

    // CANopen does not restun acc
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getJointAccelerations());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalseJointAccelerationsIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointAccelerations());
}

TEST_F(CiA402RobotShould, returnValidJointForceTorquesIfInitialized) {
    JointForceTorques jT(6);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointForceTorques().value(), jT));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalseJointForceTorquesIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointForceTorques());
}

TEST_F(CiA402RobotShould, returnFalseTaskPosesIfInitialized) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskPose());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalseTaskPosesIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskPose());
}

TEST_F(CiA402RobotShould, returnFalseTaskVelocityIfInitialized) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalseTaskVelocityIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskVelocity());
}

TEST_F(CiA402RobotShould, returnFalseTaskAccelerationIfInitialized) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskAcceleration());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalseTaskAccelerationIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskAcceleration());
}

TEST_F(CiA402RobotShould, returnFalseTaskForceTorqueIfInitialized) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskForceTorque());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalseTaskForceTorqueIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskForceTorque());
}

TEST_F(CiA402RobotShould, returnTrueIfFeasibleSetJointPositionsInput) {
    std::vector<JointPositions> q {
        JointPositions({0, 0, 0, 0, 0, 0}),
        JointPositions({1, 2, 3, 4, 5, 6}),
        JointPositions({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointPositions(true, q[i]));
    }
    for (int i = 0; i < q.size(); i++) {
        ASSERT_TRUE(sut_->setJointPositions(false, q[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnTrueIfSetJointPositionsCorrectly) {
    std::vector<JointPositions> q {
        JointPositions({0, 0, 0, 0, 0, 0}),
        JointPositions({1, 2, 3, 4, 5, 6}),
        JointPositions({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6}) };

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

TEST_F(CiA402RobotShould, returnTrueIfSetJointVelocitySetCorrectly) {
    std::vector<JointVelocities> q {
        JointVelocities({0, 0, 0, 0, 0, 0}),
        JointVelocities({1, 2, 3, 4, 5, 6}),
        JointVelocities({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6}) };

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

TEST_F(CiA402RobotShould, returnTrueIfSetJointForceTorquesSetCorrectly) {
    std::vector<JointForceTorques> t {
        JointForceTorques({0, 0, 0, 0, 0, 0}),
        JointForceTorques({1, 2, 3, 4, 5, 6}),
        JointForceTorques({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6}) };

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

TEST_F(CiA402RobotShould, returnFalsesetTaskPoseIfInitialized) {
    TaskPose dummy({1, 0, 0}, crf::math::rotation::CardanXYZ({0, 0, 0}));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setTaskPose(true, dummy));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalsesetTaskVelocityIfInitialized) {
    TaskVelocity dummy {1, 0, 0, 0, 0, 0};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setTaskVelocity(true, dummy));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnFalsesetTaskForceTorqueIfInitialized) {
    TaskForceTorque dummy {1, 0, 0, 0, 0, 0};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setTaskForceTorque(true, dummy));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnTrueIfSetProfileJointVelocitiesCorrectly) {
    std::vector<JointVelocities> qd {
        JointVelocities({0, 0, 0, 0, 0, 0}),
        JointVelocities({1, 2, 3, 4, 5, 6}),
        JointVelocities({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6}) };

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

TEST_F(CiA402RobotShould, returnTrueIfSetProfileJointAccelerationsCorrectly) {
    std::vector<JointAccelerations> qdd {
        JointAccelerations({0, 0, 0, 0, 0, 0}),
        JointAccelerations({1, 2, 3, 4, 5, 6}),
        JointAccelerations({-1.1, 2.5, -3.6, 4.12345, -5.35682, 6}) };

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

TEST_F(CiA402RobotShould, returnTureIfSoftStopped) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->softStop());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(CiA402RobotShould, returnTureIfHardStopped) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->hardStop());
    ASSERT_TRUE(sut_->deinitialize());
}
