/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <KinovaTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <kdl/frames.hpp>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>

#include "KinovaArm/KinovaApiInterfaceMock.hpp"

#include "EventLogger/EventLogger.hpp"
#include "RobotArm/IRobotArm.hpp"
#include "KinovaArm/KinovaJaco.hpp"

#define KINOVA_JACO_NUM_JOINTS 6

using crf::actuators::kinovaarm::KinovaApiInterfaceMock;
using crf::actuators::robotarm::IRobotArm;
using crf::actuators::kinovaarm::KinovaJaco;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointForceTorques;

using testing::_;
using testing::Invoke;
using testing::NiceMock;

class KinovaJacoShould: public ::testing::Test {
 protected:
    KinovaJacoShould(): logger_("KinovaJacoShould"), controlMode_(0) {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        apiInterfaceMock_.reset(new NiceMock<KinovaApiInterfaceMock>);
        ON_CALL(*apiInterfaceMock_, ethernetInitEthernetAPI(_)).
            WillByDefault(Invoke([this](EthernetCommConfig& config) {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetCloseAPI()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetStartControlAPI()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetStopControlAPI()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetRefresDevicesList()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetGetDevices(_, _)).
            WillByDefault(Invoke([this](KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) {
                return testAddDevicesList(devices, result);
            }));
        ON_CALL(*apiInterfaceMock_, ethernetSetActiveDevice(_)).
            WillByDefault(Invoke([this](KinovaDevice device) {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetMoveHome()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetInitFingers()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetEraseAllTrajectories()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetActivateSingularityAutomaticAvoidance(_)).
            WillByDefault(Invoke([this](int state) {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetActivateCollisionAutomaticAvoidance(_)).
            WillByDefault(Invoke([this](int state) {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetGetAngularPosition(_)).
            WillByDefault(Invoke([this](AngularPosition& response) {
                testFakeJointPositions(response);
                return 1;
            }));
        ON_CALL(*apiInterfaceMock_, ethernetGetAngularVelocity(_)).
            WillByDefault(Invoke([this](AngularPosition& response) {
                testFakeJointVelocities(response);
                return 1;
            }));
        ON_CALL(*apiInterfaceMock_, ethernetGetAngularForce(_)).
            WillByDefault(Invoke([this](AngularPosition& response) {
                testFakeJointForceTorques(response);
                return 1;
            }));
        ON_CALL(*apiInterfaceMock_, ethernetGetAngularForceGravityFree(_)).
                WillByDefault(Invoke([this](AngularPosition& response) {
            testFakeJointForceTorques(response);
            return 1;
        }));
        ON_CALL(*apiInterfaceMock_, ethernetGetTaskPose(_)).
            WillByDefault(Invoke([this](CartesianPosition& response) {
                testFakeArmPosition(response);
                return 1;
            }));
        ON_CALL(*apiInterfaceMock_, ethernetSendAdvanceTrajectory(_)).
            WillByDefault(Invoke([this](const TrajectoryPoint& trajectory) {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetSetTaskControl()).
            WillByDefault(Invoke([this] {
                controlMode_ = 0;
                return 1;
            }));
        ON_CALL(*apiInterfaceMock_, ethernetSetAngularControl()).
            WillByDefault(Invoke([this] {
                controlMode_ = 1;
                return 1;
            }));
        ON_CALL(*apiInterfaceMock_, ethernetMoveHome()).
                WillByDefault(Invoke([this] {
            return 1;
        }));
        ON_CALL(*apiInterfaceMock_, ethernetGetControlType(_)).
            WillByDefault(Invoke([this](int& response) {
                response = controlMode_;
                return 1;
            }));
        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("KinovaJacoTests.cpp"));
        testDirName_ += "config/";
    }
    ~KinovaJacoShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    int testAddDevicesList(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) {  // NOLINT
        result = 1;
        snprintf(devices[0].SerialNumber, SERIAL_LENGTH, "SomeBullshit");
        snprintf(devices[1].SerialNumber, SERIAL_LENGTH, "PJ00900006163400002");
        snprintf(devices[2].SerialNumber, SERIAL_LENGTH, "PJ00900006509031-0 ");
        return 3;
    }
    void testFakeJointPositions(AngularPosition& position) { // NOLINT
        position.Actuators.Actuator1 = -90;
        position.Actuators.Actuator2 = 90;
        position.Actuators.Actuator3 = 90;
        position.Actuators.Actuator4 = 90;
        position.Actuators.Actuator5 = 90;
        position.Actuators.Actuator6 = -90;
    }
    void testFakeJointVelocities(AngularPosition& velocity) { // NOLINT
        velocity.Actuators.Actuator1 = -5;
        velocity.Actuators.Actuator2 = 5;
        velocity.Actuators.Actuator3 = 5;
        velocity.Actuators.Actuator4 = 5;
        velocity.Actuators.Actuator5 = 5;
        velocity.Actuators.Actuator6 = -5;
    }
    void testFakeJointForceTorques(AngularPosition& torque) { // NOLINT
        torque.Actuators.Actuator1 = -5;
        torque.Actuators.Actuator2 = 5;
        torque.Actuators.Actuator3 = 5;
        torque.Actuators.Actuator4 = 5;
        torque.Actuators.Actuator5 = 5;
        torque.Actuators.Actuator6 = -5;
    }
    void testFakeArmPosition(CartesianPosition& position) { // NOLINT
        position.Coordinates.X = -0.3f;
        position.Coordinates.Y = -0.3f;
        position.Coordinates.Z = 0.2f;
        position.Coordinates.ThetaX = 0.1f;
        position.Coordinates.ThetaY = 0.2f;
        position.Coordinates.ThetaZ = 0.2f;
    }
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<KinovaApiInterfaceMock> > apiInterfaceMock_;
    std::unique_ptr<KinovaJaco> sut_;
    std::string testDirName_;
    int controlMode_;
};

TEST_F(KinovaJacoShould, returnFalseIfInitializedOrDeinitializedTwice) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(KinovaJacoShould, returnFalseWhenIncorrectConfigFileProvided) {
    std::ifstream robotData_badKey(testDirName_ + "configFile_badKey.json");
    nlohmann::json robotJSON_badKey = nlohmann::json::parse(robotData_badKey);
    sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON_badKey));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(KinovaJacoShould, returnEmptyVectorWhenJointsGetterInvokedAndNotInitialized) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_EQ(boost::none, sut_->getJointPositions());
    ASSERT_EQ(boost::none, sut_->getJointVelocities());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(KINOVA_JACO_NUM_JOINTS, sut_->getJointPositions()->size());
    ASSERT_EQ(KINOVA_JACO_NUM_JOINTS, sut_->getJointVelocities()->size());
}

TEST_F(KinovaJacoShould, returnEmptyMatrixWhenArmPositionGetterInvokeAndNotInitialized) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_FALSE(sut_->getTaskPose().is_initialized());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->getTaskPose().is_initialized());
}

TEST_F(KinovaJacoShould, returnFalseWhenStopperInvokeAndNotInitialized) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_FALSE(sut_->stopArm());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->stopArm());
}

TEST_F(KinovaJacoShould, returnJointPositionsIfApiCallSuccessful) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    JointPositions expectedJointPositions({M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2});
    auto result = sut_->getJointPositions().get();
    for (int i=0; i < KINOVA_JACO_NUM_JOINTS; i++) {
        ASSERT_NEAR(expectedJointPositions[i], result[i], 1e-5);
    }
}

/*
 *  Now we are returning the actual values of the kinova velocity so this test has
 *  to be changed. 
 *  (jplayang)
 */

TEST_F(KinovaJacoShould, DISABLED_returnJointVelocitiesIfApiCallSuccessful) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    JointVelocities expectedJointVelocities({M_PI/18, M_PI/18, M_PI/18, M_PI/18, M_PI/18, M_PI/18});
    // because of API bug our Kinova impl always returns previously set velocity
    ASSERT_TRUE(sut_->setJointVelocities(expectedJointVelocities));
    ASSERT_TRUE(areAlmostEqual(expectedJointVelocities, sut_->getJointVelocities().get(), 10e-5));
}

TEST_F(KinovaJacoShould, returnJointForceTorquesIfApiCallSuccessful) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    JointForceTorques expectedJointForceTorques({5, 5, 5, 5, 5, 5});
    ASSERT_TRUE(
        areAlmostEqual(expectedJointForceTorques, sut_->getJointForceTorques().get(), 10e-5));
}

TEST_F(KinovaJacoShould, returnArmPositionIfApiCallSuccessful) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::TaskPose taskPosBad(
        Eigen::Vector3d::Zero(), crf::math::rotation::CardanXYZ({0.0, 0.0, 0.0}));
    ASSERT_FALSE(areAlmostEqual(taskPosBad, sut_->getTaskPose().get(), 10e-5));
    crf::utility::types::TaskPose taskPosGood(
        {-0.3f, -0.3f, 0.2f},
        crf::math::rotation::CardanXYZ({0.1f, 0.2f, 0.2f}));
    ASSERT_TRUE(areAlmostEqual(taskPosGood, sut_->getTaskPose().get(), 10e-5));
}

TEST_F(KinovaJacoShould, returnFalseWhenSettersInvokeAndNotInitialized) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    JointVelocities jointVelocities({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_FALSE(sut_->setJointVelocities(jointVelocities));
    JointPositions jointPositions({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_FALSE(sut_->setJointPositions(jointPositions));
}

TEST_F(KinovaJacoShould, returnFalseIfWrongInputJointsValuesSetJointVelocities) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    JointVelocities jointVelocities({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_TRUE(sut_->setJointVelocities(jointVelocities));
    jointVelocities[0] = 100;
    ASSERT_FALSE(sut_->setJointVelocities(jointVelocities));
}

TEST_F(KinovaJacoShould, returnFalseIfMoveHomeNotInvokedBeforeSetJointPositions) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    JointPositions jointPositions({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_FALSE(sut_->setJointPositions(jointPositions));
    ASSERT_TRUE(sut_->moveHomePosition());
    ASSERT_TRUE(sut_->setJointPositions(jointPositions));
}

TEST_F(KinovaJacoShould, returnFalseIfWrongInputJointsValuesSetJointPositions) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->moveHomePosition());
    JointPositions jointPositions({0.1, 0, 0, 0, 0, 0.2});
    ASSERT_TRUE(sut_->setJointPositions(jointPositions));
    jointPositions[0] = 100;
    ASSERT_FALSE(sut_->setJointPositions(jointPositions));
}

TEST_F(KinovaJacoShould, returnFalseOrNonForNotAvailableInterfaceMethods) {
    std::ifstream robotData(testDirName_ + "goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new KinovaJaco(apiInterfaceMock_, robotJSON)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_FALSE(sut_->setJointPositions(sut_->getJointPositions().get()));
    ASSERT_FALSE(sut_->setJointForceTorques(JointForceTorques(KINOVA_JACO_NUM_JOINTS)));
    ASSERT_FALSE(sut_->setTaskPose(sut_->getTaskPose().get()));
    ASSERT_FALSE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity(), false));
    ASSERT_FALSE(sut_->enableBrakes());
    ASSERT_FALSE(sut_->disableBrakes());
}
