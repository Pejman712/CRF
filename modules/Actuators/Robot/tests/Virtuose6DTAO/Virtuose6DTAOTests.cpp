/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <fstream>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "Robot/Virtuose6DTAO/Virtuose6DTAO.hpp"
#include "Haption/HaptionAPIMock.hpp"
#include "crf/ResponseDefinitions.hpp"

using testing::_;
using testing::AnyNumber;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::SaveArg;
using testing::Return;
using ::testing::An;

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;

class Virtuose6DTAOShould: public ::testing::Test {
 protected:
    Virtuose6DTAOShould():
        logger_("Virtuose6DTAOShould") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("modules"));
        std::string configFilePath(testDirName +
            "modules/Actuators/Robot/tests/Virtuose6DTAO/config/Virtuose6DTAO.json");
        std::ifstream configIfStream(configFilePath);
        robotConfigFile_ = nlohmann::json::parse(configIfStream);
        haptionMock_.reset(new NiceMock<crf::devices::haption::HaptionAPIMock>);
    }

    ~Virtuose6DTAOShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        resultCode_ = crf::Code::OK;
        resultJoints_ = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

        resultCartesian_.t_x = 1.0;
        resultCartesian_.t_y = 2.0;
        resultCartesian_.t_z = 3.0;
        resultCartesian_.r_x = 4.0;
        resultCartesian_.r_y = 5.0;
        resultCartesian_.r_z = 6.0;

        resultPose_.t_x = 1.0;
        resultPose_.t_y = 2.0;
        resultPose_.t_z = 3.0;
        resultPose_.q_w = 1.0;
        resultPose_.q_x = 0.0;
        resultPose_.q_y = 0.0;
        resultPose_.q_z = 0.0;

        brakes_ = HAPTION::BrakeStatus::BRAKE_RELEASED;

        receivedJointPositions_ = JointPositions({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        receivedJointVelocities_ = JointVelocities({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        ON_CALL(*haptionMock_, startConnection()).WillByDefault(
            Invoke([this] {
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, stopConnection()).WillByDefault(
            Invoke([this] {
            return resultCode_;
        }));

        ON_CALL(*haptionMock_, startCartesianPositionMode()).WillByDefault(
            Invoke([this] {
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, startJointPositionMode()).WillByDefault(
            Invoke([this] {
            return resultCode_;
        }));

        ON_CALL(*haptionMock_, getJointAngles(_)).WillByDefault(
            Invoke([this](HAPTION::JointVector& oAngles) {
            oAngles = resultJoints_;
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, getJointSpeeds(_)).WillByDefault(
            Invoke([this](HAPTION::JointVector& oSpeeds) {
            oSpeeds = resultJoints_;
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, getJointTorques(_)).WillByDefault(
            Invoke([this](HAPTION::JointVector& oTorques) {
            oTorques = resultJoints_;
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, getCartesianPose(An<HAPTION::Displacement&>())).WillByDefault(
            Invoke([this](HAPTION::Displacement& oPose) {
            oPose = resultPose_;
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, getCartesianSpeed(_)).WillByDefault(
            Invoke([this](HAPTION::CartesianVector& oSpeeds) {
            oSpeeds = resultCartesian_;
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, getCartesianForce(_)).WillByDefault(
            Invoke([this](HAPTION::CartesianVector& oForce) {
            oForce = resultCartesian_;
            return resultCode_;
        }));

        ON_CALL(*haptionMock_, setJointAngles(_)).WillByDefault(
            Invoke([this](HAPTION::JointVector const& p) {
            receivedJointPositions_ = JointPositions(
                {p.v[0], p.v[1], p.v[2], p.v[3], p.v[4], p.v[5], p.v[6]});
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, setJointSpeeds(_)).WillByDefault(
            Invoke([this](HAPTION::JointVector const& v) {
            receivedJointVelocities_ = JointVelocities(
                {v.v[0], v.v[1], v.v[2], v.v[3], v.v[4], v.v[5], v.v[6]});
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, addJointTorqueOverlay(_)).WillByDefault(
            Invoke([this](HAPTION::JointVector const& t) {
            receivedJointForceTorques_ = JointForceTorques(
                {t.v[0], t.v[1], t.v[2], t.v[3], t.v[4], t.v[5], t.v[6]});
            return resultCode_;
        }));

        ON_CALL(*haptionMock_, setCartesianPose(An<HAPTION::Displacement const&>())).WillByDefault(
            Invoke([this](HAPTION::Displacement const& p) {
            receivedTaskPosition_ = TaskPose(
                Eigen::Vector3d(p.t_x, p.t_y, p.t_z),
                Eigen::Quaterniond(p.q_w, p.q_x, p.q_y, p.q_z));
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, setCartesianSpeed(_)).WillByDefault(
            Invoke([this](HAPTION::CartesianVector const& v) {
            receivedTaskVelocity_ = TaskVelocity({v.t_x, v.t_y, v.t_z, v.r_x, v.r_y, v.r_z});
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, addCartesianForceOverlay(_)).WillByDefault(
            Invoke([this](HAPTION::CartesianVector const& f) {
            receivedTaskForceTorque_ = TaskForceTorque({f.t_x, f.t_y, f.t_z, f.r_x, f.r_y, f.r_z});
            return resultCode_;
        }));

        ON_CALL(*haptionMock_, getBrakeStatus(_)).WillByDefault(
            Invoke([this](HAPTION::BrakeStatus& oStatus) {
            oStatus = brakes_;
            return resultCode_;
        }));
        ON_CALL(*haptionMock_, changeBrakeStatus(_)).WillByDefault(
            Invoke([this](HAPTION::BrakeStatus const& iStatus) {
            brakes_ = iStatus;
            return resultCode_;
        }));

        ON_CALL(*haptionMock_, clearError()).WillByDefault(
            Invoke([this] {
            return resultCode_;
        }));

        sut_.reset(new crf::actuators::robot::Virtuose6DTAO(haptionMock_,
            crf::actuators::robot::Virtuose6DTAOConfiguration(robotConfigFile_)));
    }

    crf::utility::logger::EventLogger logger_;
    nlohmann::json robotConfigFile_;
    std::shared_ptr<NiceMock<crf::devices::haption::HaptionAPIMock>> haptionMock_;
    std::unique_ptr<crf::actuators::robot::Virtuose6DTAO> sut_;

    crf::Code resultCode_;
    HAPTION::JointVector resultJoints_;
    HAPTION::CartesianVector resultCartesian_;
    HAPTION::Displacement resultPose_;
    HAPTION::BrakeStatus brakes_;
    JointPositions receivedJointPositions_;
    JointVelocities receivedJointVelocities_;
    JointForceTorques receivedJointForceTorques_;
    TaskPose receivedTaskPosition_;
    TaskVelocity receivedTaskVelocity_;
    TaskForceTorque receivedTaskForceTorque_;
};

TEST_F(Virtuose6DTAOShould, returnTrueIfInitOnceAndDeInitOnce) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(Virtuose6DTAOShould, returnFalseIfFailedToConnectOrDisconnect) {
    resultCode_ = crf::Code::Disconnected;
    ASSERT_FALSE(sut_->initialize());
    resultCode_ = crf::Code::OK;
    ASSERT_TRUE(sut_->initialize());
    resultCode_ = crf::Code::Disconnected;
    ASSERT_FALSE(sut_->deinitialize());
    resultCode_ = crf::Code::OK;
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(Virtuose6DTAOShould, returnFalseIfNotImplementedMethods) {
    std::array<double, 3> gravity({1.0, 2.0, 3.0});
    ASSERT_EQ(sut_->setGravity(gravity).get_response(), crf::Code::NotImplemented);
}

TEST_F(Virtuose6DTAOShould, returnFalseIfNotAllowedMethods) {
    ASSERT_EQ(sut_->getJointAccelerations().get_response(), crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->getTaskAcceleration().get_response(), crf::Code::MethodNotAllowed);

    JointPositions jointPos({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    JointVelocities jointVel({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    JointAccelerations jointAcc({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    ASSERT_EQ(sut_->setJointPositions(true, jointPos, jointVel, jointAcc).get_response(),
        crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->setJointVelocities(true, jointVel, jointAcc).get_response(),
        crf::Code::MethodNotAllowed);

    TaskPose taskPose({0.0, 0.0, 0.0}, CardanXYZ({0.0, 0.0, 0.0}));
    TaskVelocity taskVel({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    TaskAcceleration taskAcc({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    ASSERT_EQ(sut_->setTaskPose(true, taskPose, taskVel, taskAcc).get_response(),
        crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->setTaskVelocity(true, taskVel, taskAcc).get_response(),
        crf::Code::MethodNotAllowed);

    ASSERT_EQ(sut_->getProfileJointVelocities().get_response(), crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->getProfileJointAccelerations().get_response(), crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->getProfileTaskVelocity().get_response(), crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->getProfileTaskAcceleration().get_response(), crf::Code::MethodNotAllowed);

    ASSERT_EQ(sut_->setProfileJointVelocities(jointVel).get_response(),
        crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->setProfileJointAccelerations(jointAcc).get_response(),
        crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->setProfileTaskVelocity(taskVel).get_response(),
        crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->setProfileTaskAcceleration(taskAcc).get_response(),
        crf::Code::MethodNotAllowed);

    ASSERT_EQ(sut_->softStop().get_response(), crf::Code::MethodNotAllowed);
    ASSERT_EQ(sut_->hardStop().get_response(), crf::Code::MethodNotAllowed);
}

TEST_F(Virtuose6DTAOShould, returnFalseInGetterIfNotInitialized) {
    ASSERT_EQ(sut_->getJointPositions().get_response(), crf::Code::NotInitialized);
    ASSERT_EQ(sut_->getJointVelocities().get_response(), crf::Code::NotInitialized);
    ASSERT_EQ(sut_->getJointForceTorques().get_response(), crf::Code::NotInitialized);
    ASSERT_EQ(sut_->getTaskPose().get_response(), crf::Code::NotInitialized);
    ASSERT_EQ(sut_->getTaskVelocity().get_response(), crf::Code::NotInitialized);
    ASSERT_EQ(sut_->getTaskForceTorque().get_response(), crf::Code::NotInitialized);
    ASSERT_EQ(sut_->getBrakes().get_response(), crf::Code::NotInitialized);
}

TEST_F(Virtuose6DTAOShould, returnFalseInSetterIfNotInitialized) {
    JointForceTorques jointForceTorques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    ASSERT_EQ(sut_->setJointForceTorques(true, jointForceTorques).get_response(),
        crf::Code::NotInitialized);

    TaskForceTorque taskForceTorque({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    ASSERT_EQ(sut_->setTaskForceTorque(true, taskForceTorque).get_response(),
        crf::Code::NotInitialized);

    std::vector<bool> brakesStatus({true, true, true, true, true, true});
    ASSERT_EQ(sut_->setBrakes(brakesStatus).get_response(), crf::Code::NotInitialized);
}

TEST_F(Virtuose6DTAOShould, returnValidJointPositionsIfInitialized) {
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->getJointPositions().get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions().value(),
        JointPositions({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0})));
}

TEST_F(Virtuose6DTAOShould, returnValidJointVelocitiesIfInitialized) {
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->getJointVelocities().get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_TRUE(areAlmostEqual(sut_->getJointVelocities().value(),
        JointVelocities({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0})));
}

TEST_F(Virtuose6DTAOShould, returnValidJointForceTorquesIfInitialized) {
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->getJointForceTorques().get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_TRUE(areAlmostEqual(sut_->getJointForceTorques().value(),
        JointForceTorques({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0})));
}

TEST_F(Virtuose6DTAOShould, returnValidTaskPoseIfInitialized) {
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->getTaskPose().get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskPose().value(),
        TaskPose({1.0, 2.0, 3.0}, Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0}))));
}

TEST_F(Virtuose6DTAOShould, returnValidTaskVelocityIfInitialized) {
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->getTaskVelocity().get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskVelocity().value(),
        TaskVelocity({1.0, 2.0, 3.0, 4.0, 5.0, 6.0})));
}

TEST_F(Virtuose6DTAOShould, returnValidTaskForceTorqueIfInitialized) {
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->getTaskForceTorque().get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskForceTorque().value(),
        TaskVelocity({1.0, 2.0, 3.0, 4.0, 5.0, 6.0})));
}

TEST_F(Virtuose6DTAOShould, returnValidBrakeStatusIfInitialized) {
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->getBrakes().get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    brakes_ = HAPTION::BrakeStatus::BRAKE_ENGAGED;
    ASSERT_EQ(sut_->getBrakes().value(),
        std::vector<bool>({true, true, true, true, true, true}));

    brakes_ = HAPTION::BrakeStatus::BRAKE_RELEASED;
    ASSERT_EQ(sut_->getBrakes().value(),
        std::vector<bool>({false, false, false, false, false, false}));
}

TEST_F(Virtuose6DTAOShould, executeSetJointForceTorquesPipelineIfInitialized) {
    JointForceTorques jointForceTorques({2.0, 3.3, 5.0, 22.0, 0.8, 5.0, 7.0});
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->setJointForceTorques(false, jointForceTorques).get_response(),
        crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_EQ(sut_->setJointForceTorques(false, jointForceTorques).get_response(),
        crf::Code::OK);

    ASSERT_TRUE(areAlmostEqual(receivedJointPositions_, JointPositions({
        resultJoints_.v[0], resultJoints_.v[1], resultJoints_.v[2], resultJoints_.v[3],
        resultJoints_.v[4], resultJoints_.v[5], resultJoints_.v[6]})));
    ASSERT_TRUE(areAlmostEqual(receivedJointVelocities_, JointVelocities({
        resultJoints_.v[0], resultJoints_.v[1], resultJoints_.v[2], resultJoints_.v[3],
        resultJoints_.v[4], resultJoints_.v[5], resultJoints_.v[6]})));

    // Due to the Haption type conversions we lose presicion on the values
    ASSERT_TRUE(areAlmostEqual(receivedJointForceTorques_, jointForceTorques, 1e-7));
}

TEST_F(Virtuose6DTAOShould, executeSetTaskForceTorquePipelineIfInitialized) {
    TaskForceTorque taskForceTorque({2.0, 3.3, 5.0, 22.0, 0.8, 5.0});
    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->setTaskForceTorque(false, taskForceTorque).get_response(),
        crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_EQ(sut_->setTaskForceTorque(false, taskForceTorque).get_response(),
        crf::Code::OK);

    ASSERT_TRUE(areAlmostEqual(receivedTaskPosition_, TaskPose(
        Eigen::Vector3d(resultPose_.t_x, resultPose_.t_y, resultPose_.t_z),
        Eigen::Quaterniond(resultPose_.q_w, resultPose_.q_x, resultPose_.q_y, resultPose_.q_z))));
    ASSERT_TRUE(areAlmostEqual(receivedTaskVelocity_, TaskVelocity({
        resultCartesian_.t_x, resultCartesian_.t_y, resultCartesian_.t_z,
        resultCartesian_.r_x, resultCartesian_.r_y, resultCartesian_.r_z})));

    // Due to the Haption type conversions we lose presicion on the values
    ASSERT_TRUE(areAlmostEqual(receivedTaskForceTorque_, taskForceTorque, 1e-7));
}

TEST_F(Virtuose6DTAOShould, executeSetBrakesIfInitialized) {
    std::vector<bool> wrongSizeBrakes({true, true});
    std::vector<bool> wrongDifferentValuesBrakes({true, true, false, true, true, true});
    std::vector<bool> goodBrakesTrue({true, true, true, true, true, true});
    std::vector<bool> goodBrakesFalse({false, false, false, false, false, false});
    ASSERT_TRUE(sut_->initialize());

    ASSERT_EQ(sut_->setBrakes(wrongSizeBrakes).get_response(), crf::Code::BadRequest);
    ASSERT_EQ(sut_->setBrakes(wrongSizeBrakes).get_response(), crf::Code::BadRequest);

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->setBrakes(goodBrakesTrue).get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_EQ(sut_->setBrakes(goodBrakesTrue).get_response(), crf::Code::OK);
    ASSERT_EQ(brakes_, HAPTION::BrakeStatus::BRAKE_ENGAGED);
    ASSERT_EQ(sut_->setBrakes(goodBrakesFalse).get_response(), crf::Code::OK);
    ASSERT_EQ(brakes_, HAPTION::BrakeStatus::BRAKE_RELEASED);
}

TEST_F(Virtuose6DTAOShould, executeResetFaultStateIfInitialized) {
    ASSERT_EQ(sut_->resetFaultState().get_response(), crf::Code::NotInitialized);

    ASSERT_TRUE(sut_->initialize());

    resultCode_ = crf::Code::Disconnected;
    ASSERT_EQ(sut_->resetFaultState().get_response(), crf::Code::Disconnected);

    resultCode_ = crf::Code::OK;
    ASSERT_EQ(sut_->resetFaultState().get_response(), crf::Code::OK);
}
