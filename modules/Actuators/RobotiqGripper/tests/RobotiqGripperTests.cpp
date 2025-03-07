/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <boost/optional.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <thread>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "Gripper/IGripper.hpp"
#include "RobotiqGripper/RobotiqGripper.hpp"

using crf::utility::logger::EventLogger;
using crf::robots::gripper::IGripper;
using crf::robots::robotiqgripper::RobotiqGripper;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

class RobotiqGripperShould: public ::testing::Test {
 protected:
    RobotiqGripperShould():
            logger_("RobotiqGripperShould"),
            devAddress_("/dev/ttyUSB0") {
        logger_->info("{} BEGIN",
                      testing::UnitTest::GetInstance()->current_test_info()->name());
        sut_.reset(new RobotiqGripper(devAddress_));
    }
    ~RobotiqGripperShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    EventLogger logger_;
    std::unique_ptr<RobotiqGripper> sut_;
    std::string devAddress_;
    const int equalityThreshold = 5;
};

TEST_F(RobotiqGripperShould, DISABLED_returnFalseWhenDoubleInitializedDeinitialized) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_getPositionReturnPosition) {
    ASSERT_FALSE(sut_->getPosition());  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->getPosition());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_setPositionPerformsMovement) {
    float goalPosition = 50;
    ASSERT_FALSE(sut_->setPosition(goalPosition));  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setPosition(goalPosition));
    // waits to perform movement
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_NEAR(sut_->getPosition().get(), goalPosition, equalityThreshold);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_setPositionOpensCloses) {
    ASSERT_FALSE(sut_->getPosition());  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());

    // open
    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Open));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_NEAR(sut_->getPosition().get(), IGripper::Gripper_Open, equalityThreshold);

    // close
    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Closed));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // the  x3 is because the gripper cant close fully because of the tip
    ASSERT_NEAR(sut_->getPosition().get(), IGripper::Gripper_Closed, equalityThreshold * 3);

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_setPositionChecksBounds) {
    ASSERT_FALSE(sut_->setPosition(50));  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setPosition(102));
    ASSERT_FALSE(sut_->setPosition(-3));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_setGraspingForceChecksBounds) {
    ASSERT_FALSE(sut_->setGraspingForce(50));  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setGraspingForce(102));
    ASSERT_FALSE(sut_->setGraspingForce(-3));
    ASSERT_TRUE(sut_->setGraspingForce(50));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_stopGripperStopsMovement) {
    ASSERT_FALSE(sut_->stopGripper());  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Open));
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    ASSERT_TRUE(sut_->stopGripper());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_setVelocityReturnsFalse) {
    ASSERT_FALSE(sut_->setVelocity(50));  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setVelocity(102));
    ASSERT_FALSE(sut_->setVelocity(-103));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_setVelocityNegativeValueOpens) {
    ASSERT_FALSE(sut_->setVelocity(-10));  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setVelocity(-1));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_setVelocityPositiveValueCloses) {
    ASSERT_FALSE(sut_->setVelocity(10));  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setVelocity(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ASSERT_TRUE(sut_->deinitialize());
}

// For this test make sure there is nothing between the gripper tips
TEST_F(RobotiqGripperShould, DISABLED_isGraspingReturnsFalse) {
    ASSERT_FALSE(sut_->isGrasping());  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Open));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_FALSE(sut_->isGrasping());

    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Closed));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_FALSE(sut_->isGrasping());

    ASSERT_TRUE(sut_->deinitialize());
}

// For this test PUT SOMETHING between the gripper tips
TEST_F(RobotiqGripperShould, DISABLED_isGraspingReturnsTrue) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Open));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_FALSE(sut_->isGrasping());

    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Closed));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_TRUE(sut_->isGrasping());

    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(RobotiqGripperShould, DISABLED_setGraspingForceReturnsFalse) {
    ASSERT_FALSE(sut_->setGraspingForce(50));  // NOT INITIALIZED
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setGraspingForce(102));
    ASSERT_FALSE(sut_->setGraspingForce(-3));

    ASSERT_TRUE(sut_->deinitialize());
}

// Put something between the gripper and try to pull out
// higher force -> harder to pull out
TEST_F(RobotiqGripperShould, DISABLED_setGraspingForceReturnsTrue) {
    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Open));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_TRUE(sut_->setGraspingForce(100));
    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Closed));
    std::this_thread::sleep_for(std::chrono::seconds(2));

    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Open));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ASSERT_TRUE(sut_->setGraspingForce(5));
    ASSERT_TRUE(sut_->setPosition(IGripper::Gripper_Closed));
    std::this_thread::sleep_for(std::chrono::seconds(2));

    ASSERT_TRUE(sut_->deinitialize());
}
