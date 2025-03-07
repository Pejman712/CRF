/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <fstream>

#include <nlohmann/json.hpp>
#include <boost/optional.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "TIMArm/TIMArm.hpp"
#include "EtherCATDevices/EtherCATMotorMock.hpp"
#include "KinovaArm/KinovaApiInterfaceMock.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"

#define JOINTS_NUMBER 9

using testing::_;
using testing::Invoke;
using testing::NiceMock;

class TIMArmShould: public ::testing::Test {
 protected:
    TIMArmShould() :
        logger_("TIMArmShould"),
        controlMode_(0) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        etherCATMock_.reset(new NiceMock<crf::devices::ethercatdevices::EtherCATMotorMock>);
        ON_CALL(*etherCATMock_, getPosition()).WillByDefault(
            Invoke([this] {return 2;}));
        ON_CALL(*etherCATMock_, getVelocity()).WillByDefault(
            Invoke([this] {return 1;}));
        ON_CALL(*etherCATMock_, getTorque()).WillByDefault(
            Invoke([this] {return 1;}));
        ON_CALL(*etherCATMock_, setVelocity(_)).WillByDefault(
            Invoke([this](int32_t velocity) {return 2;}));
        ON_CALL(*etherCATMock_, setTorque(_)).WillByDefault(
            Invoke([this](int16_t torque) {return 1;}));
        apiInterfaceMock_.reset(new NiceMock<crf::robots::kinovaarm::KinovaApiInterfaceMock>);
        ON_CALL(*apiInterfaceMock_, ethernetInitEthernetAPI(_)).WillByDefault(
            Invoke([this](EthernetCommConfig& config) {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetCloseAPI()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetStartControlAPI()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetStopControlAPI()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetRefresDevicesList()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetGetDevices(_, _)).WillByDefault(
            Invoke([this](KinovaDevice devices[MAX_KINOVA_DEVICE], int &result) {
                return testAddDevicesList(devices, result);
            }));
        ON_CALL(*apiInterfaceMock_, ethernetSetActiveDevice(_)).WillByDefault(
            Invoke([this](KinovaDevice device) {return 1;}));
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
        ON_CALL(*apiInterfaceMock_, ethernetGetAngularPosition(_)).WillByDefault(
            Invoke([this](AngularPosition& response) {return testFakeJointPositions(response);}));
        ON_CALL(*apiInterfaceMock_, ethernetGetAngularVelocity(_)).WillByDefault(
            Invoke([this](AngularPosition& response) {return testFakeJointVelocities(response);}));
        ON_CALL(*apiInterfaceMock_, ethernetGetAngularForce(_)).WillByDefault(
            Invoke([this](AngularPosition& response) {return testFakeJointForceTorques(response);})); // NOLINT
        ON_CALL(*apiInterfaceMock_, ethernetGetAngularForceGravityFree(_)).WillByDefault(
            Invoke([this](AngularPosition& response) {return testFakeJointForceTorques(response);})); // NOLINT
        ON_CALL(*apiInterfaceMock_, ethernetGetTaskPose(_)).WillByDefault(
            Invoke([this](CartesianPosition& response) {return testFakeArmPosition(response);}));
        ON_CALL(*apiInterfaceMock_, ethernetSendAdvanceTrajectory(_)).WillByDefault(
            Invoke([this](const TrajectoryPoint& trajectory) {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetSetTaskControl()).WillByDefault(
            Invoke([this] {
                controlMode_ = 0;
                return 1;
            }));
        ON_CALL(*apiInterfaceMock_, ethernetSetAngularControl()).WillByDefault(
            Invoke([this] {
                controlMode_ = 1;
                return 1;
            }));
        ON_CALL(*apiInterfaceMock_, ethernetMoveHome()).
            WillByDefault(Invoke([this] {return 1;}));
        ON_CALL(*apiInterfaceMock_, ethernetGetControlType(_)).WillByDefault(
            Invoke([this](int& response) {
                response = controlMode_;
                return 1;
            }));
        configDirName_ = __FILE__;
        configDirName_ = configDirName_.substr(0, configDirName_.find("tests"));
        configDirName_ += "tests/config/";
    }
    ~TIMArmShould() {
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
    int testFakeJointPositions(AngularPosition& position) {  // NOLINT
        position.Actuators.Actuator1 = -90;
        position.Actuators.Actuator2 = 90;
        position.Actuators.Actuator3 = 90;
        position.Actuators.Actuator4 = 90;
        position.Actuators.Actuator5 = 90;
        position.Actuators.Actuator6 = -90;
        return 1;
    }
    int testFakeJointVelocities(AngularPosition& velocity) {  // NOLINT
        velocity.Actuators.Actuator1 = -10.0;
        velocity.Actuators.Actuator2 = 10.0;
        velocity.Actuators.Actuator3 = 10.0;
        velocity.Actuators.Actuator4 = 10.0;
        velocity.Actuators.Actuator5 = 10.0;
        velocity.Actuators.Actuator6 = -10.0;
        return 1;
    }
    int testFakeJointForceTorques(AngularPosition& torque) {  // NOLINT
        torque.Actuators.Actuator1 = -5;
        torque.Actuators.Actuator2 = 5;
        torque.Actuators.Actuator3 = 5;
        torque.Actuators.Actuator4 = 5;
        torque.Actuators.Actuator5 = 5;
        torque.Actuators.Actuator6 = -5;
        return 1;
    }
    int testFakeArmPosition(CartesianPosition& position) {  // NOLINT
        position.Coordinates.X = -0.3;
        position.Coordinates.Y = -0.3;
        position.Coordinates.Z = 0.2;
        position.Coordinates.ThetaX = 0.1;
        position.Coordinates.ThetaY = 0.2;
        position.Coordinates.ThetaZ = 0.2;
        return 1;
    }
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<crf::actuators::kinovaarm::KinovaApiInterfaceMock> > apiInterfaceMock_;
    std::shared_ptr<NiceMock<crf::devices::ethercatdevices::EtherCATMotorMock> > etherCATMock_;
    std::unique_ptr<crf::actuators::timarm::TIMArm> sut_;
    std::string configDirName_;
    int controlMode_;
};

TEST_F(TIMArmShould, returnFalseIfInitializedOrDeinitializedTwice) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(TIMArmShould, returnFalseIfWrongConfigFile) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/badNetworkConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(TIMArmShould, returnFalseIfKinovaApiNullptr) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        nullptr)));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(TIMArmShould, returnEmptyVectorWhenJointsGetterInvokedAndNotInitialized) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_EQ(boost::none, sut_->getJointPositions());
    ASSERT_EQ(boost::none, sut_->getJointVelocities());
    ASSERT_EQ(boost::none, sut_->getJointForceTorques());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_EQ(JOINTS_NUMBER, sut_->getJointPositions()->size());
    ASSERT_EQ(JOINTS_NUMBER, sut_->getJointVelocities()->size());
    ASSERT_EQ(JOINTS_NUMBER, sut_->getJointForceTorques()->size());
}

TEST_F(TIMArmShould, returnFalseWhenStopperInvokeAndNotInitialized) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_FALSE(sut_->stopArm());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->stopArm());
}

TEST_F(TIMArmShould, returnFalseWhenBrakesInvokeAndNotInitialized) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_FALSE(sut_->enableBrakes());
    ASSERT_FALSE(sut_->disableBrakes());
}

TEST_F(TIMArmShould, returnJointPositionsIfApiAndEthercatCallSuccessful) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::JointPositions expectedJointPositions(
        {2.0f, -2.0f, 2.0f, 0.0f, -M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2});
    auto result = sut_->getJointPositions().get();
    for (int i=0; i < JOINTS_NUMBER; i++) {
        ASSERT_NEAR(expectedJointPositions(i), result(i), 1e-6);
    }
}

/*
 *  The kinova now returns the real velocities so this test need a review 
 *  (jplayang)
 */
TEST_F(TIMKinovaArmShould, DISABLED_returnJointVelocitiesIfApiAndEthercatCallSuccessful) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::JointVelocities expectedJointVelocities(
        {1.0f, -1.0f, 1.0f, M_PI/18, M_PI/18, M_PI/18, M_PI/18, M_PI/18, M_PI/18});
    // Because of API bug our Kinova impl always returns previously set velocity
    ASSERT_TRUE(sut_->setJointVelocities(expectedJointVelocities));
    auto result = sut_->getJointVelocities().get();
    for (int i=0; i < JOINTS_NUMBER; i++) {
        ASSERT_NEAR(expectedJointVelocities(i), result(i), 1e-6);
    }
}

TEST_F(TIMArmShould, returnJointForceTorquesIfApiAndEthercatCallSuccessful) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::JointForceTorques expectedJointForceTorques(
        {1.0f, -1.0f, 1.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f});
    ASSERT_EQ(expectedJointForceTorques, sut_->getJointForceTorques().get());
}

TEST_F(TIMArmShould, returnFalseWhenSettersInvokeAndNotInitialized) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    crf::utility::types::JointVelocities jointVelocities(
        {0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f});
    ASSERT_FALSE(sut_->setJointVelocities(jointVelocities));
    crf::utility::types::JointForceTorques jointForceTorques(
        {0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f});
    ASSERT_FALSE(sut_->setJointForceTorques(jointForceTorques));
}

TEST_F(TIMArmShould, returnFalseIfWrongInputJointsValuesSetJointVelocities) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::JointVelocities jointVelocities(
        {0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f});
    ASSERT_TRUE(sut_->setJointVelocities(jointVelocities));
    jointVelocities(0) = 100;
    ASSERT_FALSE(sut_->setJointVelocities(jointVelocities));
}

TEST_F(TIMArmShould, returnFalseIfWrongInputJointsValuesSetJointForceTorques) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::JointForceTorques jointForceTorques(
        {50.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f});
    ASSERT_FALSE(sut_->setJointForceTorques(jointForceTorques));
}

TEST_F(TIMArmShould, returnFalseIfRequestedSetJointForceTorquesForKinova) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::JointForceTorques jointForceTorques(
        {0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f});
    //  Set torque is not supported on the Kinova
    ASSERT_FALSE(sut_->setJointForceTorques(jointForceTorques));
}

TEST_F(TIMArmShould, returnFalseInSettersIfRobotInMaxPositionLimits) {
    EXPECT_CALL(*apiInterfaceMock_, ethernetGetAngularPosition(_)).WillRepeatedly(
            Invoke([this](AngularPosition& response) {
                response.Actuators.Actuator1 = -90;
                response.Actuators.Actuator2 = 90;
                response.Actuators.Actuator3 = 90;
                response.Actuators.Actuator4 = 90;
                response.Actuators.Actuator5 = 90;
                response.Actuators.Actuator6 = -179;
                return 1;
            }));
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::JointVelocities jointVelocities(
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f});
    ASSERT_FALSE(sut_->setJointVelocities(jointVelocities));
    crf::utility::types::JointForceTorques jointForceTorques(
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f});
    ASSERT_FALSE(sut_->setJointForceTorques(jointForceTorques));
}

TEST_F(TIMArmShould, returnFalseInSettersIfRobotInMinPositionLimits) {
    EXPECT_CALL(*apiInterfaceMock_, ethernetGetAngularPosition(_)).WillRepeatedly(
            Invoke([this](AngularPosition& response) {
                response.Actuators.Actuator1 = 89;
                response.Actuators.Actuator2 = 90;
                response.Actuators.Actuator3 = 90;
                response.Actuators.Actuator4 = 90;
                response.Actuators.Actuator5 = 90;
                response.Actuators.Actuator6 = -90;
                return 1;
            }));
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    crf::utility::types::JointVelocities jointVelocities(
        {0.0f, 0.0f, 0.0f, -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    ASSERT_FALSE(sut_->setJointVelocities(jointVelocities));
    crf::utility::types::JointForceTorques jointForceTorques(
        {0.0f, 0.0f, 0.0f, -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    ASSERT_FALSE(sut_->setJointForceTorques(jointForceTorques));
}

TEST_F(TIMArmShould, returnFalseIfRequestedEnableBrakesForKinova) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    //  Enable brakes is not supported on the Kinova
    ASSERT_FALSE(sut_->enableBrakes());
}

TEST_F(TIMArmShould, returnFalseIfRequestedDisableBrakesForKinova) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    //  Disable brakes is not supported on the Kinova
    ASSERT_FALSE(sut_->disableBrakes());
}

TEST_F(TIMArmShould, returnRobotConfiguration) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_NE(sut_->getConfiguration(), nullptr);
}

TEST_F(TIMArmShould, returnFalseOrNonForNotAvailableInterfaceMethods) {
    std::ifstream robotData(configDirName_ + "Robots/TIMArm/goodConfiguration.json");
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    ASSERT_NO_THROW(sut_.reset(new crf::actuators::timarm::TIMArm(robotJSON,
        etherCATMock_,
        etherCATMock_,
        etherCATMock_,
        apiInterfaceMock_)));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->getTaskPose());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_FALSE(sut_->setJointPositions(sut_->getJointPositions().get()));
    ASSERT_FALSE(sut_->setTaskPose(crf::utility::types::TaskPose()));
    ASSERT_FALSE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity(), false));
}
