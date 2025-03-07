/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
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
#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Types/Types.hpp"
#include "UniversalRobotRTDE/UniversalRobotRTDEInterfaceMock.hpp"
#include "Robot/UniversalRobot/UniversalRobot.hpp"

using testing::_;
using testing::AnyNumber;
using testing::DoAll;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::SaveArg;
using testing::Return;

using crf::actuators::robot::UniversalRobot;
using crf::communication::universalrobotrtde::UniversalRobotRTDEInterfaceMock;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

using crf::actuators::robot::UniversalRobotConfiguration;

class UniversalRobotShould: public ::testing::Test {
 protected:
    UniversalRobotShould():
        logger_("UniversalRobotShould") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("UniversalRobot"));
        std::string configFilePath(testDirName + "UniversalRobot/config/UR10eSimulation.json");
        logger_->debug(configFilePath);
        std::ifstream configIfStream(configFilePath);
        robotConfigFile_ = nlohmann::json::parse(configIfStream);
        UniversalRobotInterfaceMock_.reset(new NiceMock<UniversalRobotRTDEInterfaceMock>);
    }

    ~UniversalRobotShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        ON_CALL(*UniversalRobotInterfaceMock_, getActualQ()).
            WillByDefault(Invoke(returnSimulatedJointPositions));
        ON_CALL(*UniversalRobotInterfaceMock_, getActualQd()).
            WillByDefault(Invoke(returnSimulatedJointVelocities));
        ON_CALL(*UniversalRobotInterfaceMock_, getActualTCPPose()).
            WillByDefault(Invoke(returnSimulatedTaskPose));
        ON_CALL(*UniversalRobotInterfaceMock_, servoJ(_, _, _, _, _, _)).
            WillByDefault(Invoke(returnSimulatedServoJResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, speedJ(_, _, _)).
            WillByDefault(Invoke(returnSimulatedSpeedJResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, moveJ(_, _, _, _)).
            WillByDefault(Invoke(returnSimulatedMoveJResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, moveL(_, _, _, _)).
            WillByDefault(Invoke(returnSimulatedMoveLResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, speedL(_, _, _)).
            WillByDefault(Invoke(returnSimulatedSpeedLResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, setGravity(_)).
            WillByDefault(Invoke(returnSimulatedSetGravityResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, zeroFtSensor()).
            WillByDefault(Return(true));
        ON_CALL(*UniversalRobotInterfaceMock_, forceMode(_, _, _, _, _)).
            WillByDefault(Invoke(returnSimulatedForceModeResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, getJointForceTorques()).
            WillByDefault(Invoke(returnSimulatedJointForceTorquesResponse));
        ON_CALL(*UniversalRobotInterfaceMock_, getActualTCPSpeed()).
            WillByDefault(Invoke(returnSimulatedTaskVelocity));
        ON_CALL(*UniversalRobotInterfaceMock_, getActualTCPForce()).
            WillByDefault(Invoke(returnSimulatedTaskForceTorque));

        sut_.reset(new UniversalRobot(
            UniversalRobotInterfaceMock_,
            UniversalRobotConfiguration(robotConfigFile_)));
    }

    static std::vector<double> returnSimulatedJointPositions() {
        std::vector<double> simJointPositions{0, 0, 0, 0, 0, 0};
        return simJointPositions;
    }

    static std::vector<double> returnSimulatedJointVelocities() {
        std::vector<double> simJointVelocities{0, 0, 0, 0, 0, 0};
        return simJointVelocities;
    }

    static std::vector<double> returnSimulatedTaskPose() {
        std::vector<double> simTaskPose{0, 0, 0, 1, 0, 0};
        return simTaskPose;
    }

    static std::vector<double> returnSimulatedJointForceTorquesResponse() {
        std::vector<double> simJointForceTorques{0, 0, 0, 0, 0, 0};
        return simJointForceTorques;
    }

    static std::vector<double> returnSimulatedTaskVelocity() {
        std::vector<double> simTaskVelocity{0, 0, 0, 0, 0, 0};
        return simTaskVelocity;
    }

    static std::vector<double> returnSimulatedTaskForceTorque() {
        std::vector<double> simTaskForce{0, 0, 0, 0, 0, 0};
        return simTaskForce;
    }

    static bool returnSimulatedServoJResponse(std::vector<double> q, double v, double a,
        double cycleTime, double lookAheadTime, double gain) {
        // Using double cast to compensate error from IEEE754 with b=2
        if (!(cycleTime >= 0.002 &&
            lookAheadTime >= static_cast<double>(static_cast<float>(0.03)) &&
            lookAheadTime <= 0.2 && gain >= 100 && gain <= 2000)) {
            return false;
        }
        return true;
    }

    static bool returnSimulatedSpeedJResponse(std::vector<double> qd, double maxAcc,
        double timeUntilFunctionReturns) {
        if (!(timeUntilFunctionReturns > 0 && maxAcc >= 0 && maxAcc <= 40)) {
            return false;
        }
        return true;
    }

    static bool returnSimulatedMoveJResponse(std::vector<double> qd, double leadingVel,
        double leadingAcc, bool async) {
        return true;
    }

    static bool returnSimulatedMoveLResponse(std::vector<double> z, double leadingVel,
        double leadingAcc, bool async) {
        return true;
    }

    static bool returnSimulatedSpeedLResponse(std::vector<double> zd, double leadingAcc,
        double loopTime) {
        return true;
    }

    static bool returnSimulatedSetGravityResponse(std::vector<double> g) {
        double gravityMagnitude = sqrt(std::pow(g[0], 2) + std::pow(g[1], 2) + std::pow(g[2], 2));
        if (std::abs(gravityMagnitude - 9.81) > 0.1) {
            return false;
        }
        return true;
    }

    static bool returnSimulatedForceModeResponse(std::vector<double> forceFrame,
        std::vector<int> complianceSelector, std::vector<double> desiredForceTorque,
        int transformTyp, std::vector<double> limits) {
        return true;
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<UniversalRobotRTDEInterfaceMock>> UniversalRobotInterfaceMock_;
    nlohmann::json robotConfigFile_;
    std::unique_ptr<UniversalRobot> sut_;
};

TEST_F(UniversalRobotShould, returnFalseIfInitializedMultipleTimes) {
    EXPECT_CALL(*UniversalRobotInterfaceMock_, initRtdeReceiveInterface(_)).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(UniversalRobotShould, returnFalseIfDeinitializedMultipleTimes) {
    EXPECT_CALL(*UniversalRobotInterfaceMock_, initRtdeReceiveInterface(_)).Times(2);
    EXPECT_CALL(*UniversalRobotInterfaceMock_, speedStop()).Times(2).WillRepeatedly(Return(true));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
}

// Test behavior for init with wrong ip adress not necesssary, because they are fixed.
TEST_F(UniversalRobotShould, returnValidJointPositionsIfInitialized) {
    JointPositions q({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointPositions().value(), q));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseJointPositionsIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointPositions());
}

TEST_F(UniversalRobotShould, returnValidJointVelocitiesIfInitialized) {
    JointVelocities qd({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointVelocities().value(), qd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseJointVelocitiesIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointVelocities());
}

TEST_F(UniversalRobotShould, returnFalseOnJointAccelerationsNotDefined) {
    ASSERT_FALSE(sut_->getJointAccelerations());
}

TEST_F(UniversalRobotShould, returnValidJointForceTorquesIfInitialized) {
    JointForceTorques jT({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getJointForceTorques().value(), jT));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnValidJointForceTorquesIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointForceTorques());
}

TEST_F(UniversalRobotShould, returnValidTaskPosesIfInitialized) {
    std::vector<double> tP{0, 0, 0, 1, 0, 0};
    double angle = std::sqrt(std::pow(tP[3], 2) + std::pow(tP[4], 2) + std::pow(tP[5], 2));
    TaskPose z(Eigen::Vector3d({tP[0], tP[1], tP[2]}),
               Eigen::AngleAxisd(angle, Eigen::Vector3d({tP[3]/angle, tP[4]/angle, tP[5]/angle})));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskPose().value(), z));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnValidTaskPosesIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskPose());
}

TEST_F(UniversalRobotShould, returnValidTaskVelocityIfInitialized) {
    std::vector<double> tV{0, 0, 0, 0, 0, 0};
    TaskVelocity zd({tV[0], tV[1], tV[2], tV[3], tV[4], tV[5]});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskVelocity().value(), zd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnValidTaskVelocityIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskVelocity());
}

TEST_F(UniversalRobotShould, returnValidTaskAccelerationIfNotDefined) {
    ASSERT_FALSE(sut_->getTaskAcceleration());
}

TEST_F(UniversalRobotShould, returnValidTaskWrenchIfInitialized) {
    std::vector<double> tT{0, 0, 0, 0, 0, 0};
    TaskForceTorque t({tT[0], tT[1], tT[2], tT[3], tT[4], tT[5]});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(areAlmostEqual(sut_->getTaskForceTorque().value(), t));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseTaskWrenchIfNotInitialized) {
    ASSERT_FALSE(sut_->getTaskForceTorque());
}

TEST_F(UniversalRobotShould, returnTrueIfFeasibleSetJointPositionsInput) {
    std::vector<JointPositions>q{JointPositions({0, 0, 0, 0, 0, 0}),
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

TEST_F(UniversalRobotShould, returnFalseIfInfeasibleSetJointPositionsInput) {
    std::vector<JointPositions> q{JointPositions({7, 0, 0, 0, 0, 0}),
        JointPositions({-7, 0, 0, 0, 0, 0}),
        JointPositions({1, 2, 3, 4, 5, 6, 0}),
        JointPositions({1, 2, 3, 4, 5}),
        JointPositions({-1.1, 2.5, -3.6, 4.12345, -7.35682, 6})};

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < q.size(); i++) {
        ASSERT_FALSE(sut_->setJointPositions(true, q[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetJointVelocitiesCalledWithoutAcceleration) {
    JointVelocities qd({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setJointVelocities(true, qd));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetJointVelocitiesCalledWithAcceleration) {
    std::vector<JointVelocities> qd{JointVelocities({0, 0, 0, 0, 0, 0}),
        JointVelocities({1, 2, 3, 3, 3, 3}),
        JointVelocities({-1.1, -2.0, -2.8, 1.23453, -2.1234, 0})};
    JointAccelerations qddLead({1, 10, 20.23, 1, 1, 1});

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qd.size(); i++) {
        ASSERT_TRUE(sut_->setJointVelocities(true, qd[i], qddLead));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseIfInfeasibleSetJointVelocitiesInput) {
    std::vector<JointVelocities> qd{JointVelocities({0, 400, 0, 0, 0, 0}),
        JointVelocities({0, -400, 0, 0, 0, 0}),
        JointVelocities({1, 2, 3, 3, 3, 3, 0}),
        JointVelocities({1, 2, 3, 3, 3})};
    JointAccelerations qddLead({1, 10, 20.23, 1, 1, 1});
    JointVelocities qdGood({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qd.size(); i++) {
        ASSERT_FALSE(sut_->setJointVelocities(true, qd[i], qddLead));
        ASSERT_FALSE(sut_->setJointVelocities(true, qd[i]));
    }
    ASSERT_FALSE(sut_->setJointVelocities(false, qdGood));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseIfSetJointForceTorquesNotImpemented) {
    JointForceTorques jT({0, 0, 0, 0, 0, 0});

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setJointForceTorques(true, jT));
    ASSERT_FALSE(sut_->setJointForceTorques(false, jT));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfFeasibleSetTaskPosesInput) {
    std::vector<TaskPose>z{TaskPose({0.5, 0.5, 0.5}, crf::math::rotation::CardanXYZ({1, 0, 0})),
        TaskPose({1, 2, 3}, crf::math::rotation::CardanXYZ({0, 1, 0})),
        TaskPose({-1.1, 2.5, -3.6}, crf::math::rotation::CardanXYZ({4.12345, -5.35682, 6}))};
    TaskVelocity zd({0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
    TaskAcceleration zdd({0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < z.size(); i++) {
        ASSERT_FALSE(sut_->setTaskPose(true, z[i]));
        ASSERT_FALSE(sut_->setTaskPose(true, z[i], zd));
        ASSERT_FALSE(sut_->setTaskPose(true, z[i], zd, zdd));
    }
    for (int i = 0; i < z.size(); i++) {
        ASSERT_TRUE(sut_->setTaskPose(false, z[i]));
        ASSERT_TRUE(sut_->setTaskPose(false, z[i], zd));
        ASSERT_TRUE(sut_->setTaskPose(false, z[i], zd, zdd));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfFeasibleSetTaskVelocityInput) {
    std::vector<TaskVelocity>zd{TaskVelocity({0.01, 0.01, 0.01, 0.01, 0.01, 0.01}),
        TaskVelocity({0.01, 0.01, 0.01, 0, 0.01, 0}),
        TaskVelocity({-0.01, 0.01, -0.06, 0.012345, -0.035682, 0.006}) };
    TaskAcceleration zdd({0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < zd.size(); i++) {
        ASSERT_TRUE(sut_->setTaskVelocity(true, zd[i]));
        ASSERT_TRUE(sut_->setTaskVelocity(true, zd[i], zdd));
    }
    for (int i = 0; i < zd.size(); i++) {
        ASSERT_FALSE(sut_->setTaskVelocity(false, zd[i]));
        ASSERT_FALSE(sut_->setTaskVelocity(false, zd[i], zdd));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfFeasibleSetTaskForceTorqueInput) {
    std::vector<TaskForceTorque> tT{TaskForceTorque({0.01, 0.01, 0.01, 0.01, 0.01, 0.01}),
        TaskForceTorque({0.01, 0.01, 0.01, 0, 0.01, 0}),
        TaskForceTorque({-0.01, 0.01, -0.06, 0.012345, -0.035682, 0.006}) };

    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < tT.size(); i++) {
        ASSERT_TRUE(sut_->setTaskForceTorque(true, tT[i]));
    }
    for (int i = 0; i < tT.size(); i++) {
        ASSERT_FALSE(sut_->setTaskForceTorque(false, tT[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetFeasibleProfileJointVelocities) {
    std::vector<JointVelocities> qd{JointVelocities({0.4, 0.4, 0.4, 0.4, 0.4, 0.4}),
        JointVelocities({1, 2, 3, 3, 3, 3}),
        JointVelocities({1.1, 2.0, 2.8, 1.23453, 2.1234, 1})};
    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qd.size(); i++) {
        ASSERT_TRUE(sut_->setProfileJointVelocities(qd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetInfeasibleProfileJointVelocities) {
    std::vector<JointVelocities> qd{JointVelocities({0, 0, 0, 0, 0, 0}),
        JointVelocities({0.4, 400, 0.4, 0.4, 0.4, 0.4}),
        JointVelocities({0.4, -0.4, 0.4, 0.4, 0.4, 0.4}),
        JointVelocities({1, 2, 3, 3, 3, 3, 0.4}),
        JointVelocities({1, 2, 3, 3, 3})};
    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qd.size(); i++) {
        ASSERT_FALSE(sut_->setProfileJointVelocities(qd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetFeasibleProfileJointAccelerations) {
    std::vector<JointAccelerations> qdd{JointAccelerations({0.4, 0.4, 0.4, 0.4, 0.4, 0.4}),
        JointAccelerations({1, 2, 3, 3, 3, 3}),
        JointAccelerations({1.1, 2.0, 2.8, 1.23453, 2.1234, 1})};
    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qdd.size(); i++) {
        ASSERT_TRUE(sut_->setProfileJointAccelerations(qdd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetInfeasibleProfileJointAccelerations) {
    std::vector<JointAccelerations> qdd{JointAccelerations({0, 0, 0, 0, 0, 0}),
        JointAccelerations({1, 400, 1, 1, 1, 1}),
        JointAccelerations({1, -1, 1, 1, 1, 1}),
        JointAccelerations({1, 2, 3, 3, 3, 3, 1}),
        JointAccelerations({1, 2, 3, 3, 3})};
    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < qdd.size(); i++) {
        ASSERT_FALSE(sut_->setProfileJointAccelerations(qdd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetFeasibleProfileTaskVelocity) {
    std::vector<TaskVelocity> zd{TaskVelocity({0.01, 0.01, 0.01, 0.01, 0.01, 0.01}),
        TaskVelocity({0.1, 0.05, 0.0523, 0.05, 0.03, 0.03}),
        TaskVelocity({0.1, 0.01, 0.08, 0.0123453, 0.021234, 0.1})};
    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < zd.size(); i++) {
        ASSERT_TRUE(sut_->setProfileTaskVelocity(zd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetInfeasibleProfileTaskVelocity) {
    std::vector<TaskVelocity> zd{TaskVelocity({0, 0, 0, 0, 0, 0}),
        TaskVelocity({0.01, 0.2, 0.01, 0.01, 0.01, 0.01}),
        TaskVelocity({0.01, -0.01, 0.01, 0.01, 0.01, 0.01})};
    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < zd.size(); i++) {
        ASSERT_FALSE(sut_->setProfileTaskVelocity(zd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetFeasibleProfileTaskAcceleration) {
    std::vector<TaskAcceleration> zdd{TaskAcceleration({0.1, 1, 0.01, 0.6, 1, 0.9}),
        TaskAcceleration({0.9, 0.05, 0.0523, 0.05, 0.03, 0.03}),
        TaskAcceleration({0.6, 0.01, 1, 0.23453, 0.234, 0.1})};
    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < zdd.size(); i++) {
        ASSERT_TRUE(sut_->setProfileTaskAcceleration(zdd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfSetInfeasibleProfileTaskAcceleration) {
    std::vector<TaskAcceleration> zdd{TaskAcceleration({0, 0.01, 0.01, 0.01, 0, 0}),
        TaskAcceleration({0.1, 1.2, 0.1, 1, 1, 1}),
        TaskAcceleration({1, -0.4, 1, 1, 1, 1})};
    ASSERT_TRUE(sut_->initialize());
    for (int i = 0; i < zdd.size(); i++) {
        ASSERT_FALSE(sut_->setProfileTaskAcceleration(zdd[i]));
    }
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseFaultStateResetNotImplemented) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->resetFaultState());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnFalseIfNotInitialized) {
    ASSERT_FALSE(sut_->softStop());
    ASSERT_FALSE(sut_->hardStop());
}

TEST_F(UniversalRobotShould, returnNoneIfNotInitialized) {
    ASSERT_FALSE(sut_->getJointVelocities());
    ASSERT_FALSE(sut_->getJointPositions());
}

TEST_F(UniversalRobotShould, returnFalseIfMethodNotImplemented) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setBrakes(std::vector<bool>(true, false)));
    ASSERT_FALSE(sut_->getBrakes());
    ASSERT_FALSE(sut_->resetFaultState());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(UniversalRobotShould, returnTrueIfFeasibleGravityVector) {
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

TEST_F(UniversalRobotShould, returnFalseIfBadGravityVector) {
    std::array<double, 3> g1{0, 0, 1};

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setGravity(g1));
    ASSERT_TRUE(sut_->deinitialize());
}
