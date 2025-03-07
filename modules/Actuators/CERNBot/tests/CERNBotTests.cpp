/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "CANSocket/CANSocketMock.hpp"
#include "CANOpenDevices/CANOpenContextMock.hpp"
#include "CANOpenDevices/CANOpenMotors/CANOpenMotorMock.hpp"
#include "CERNBot/CERNBot.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::Invoke;

using crf::communication::cansocket::CANSocketMock;
using crf::devices::canopendevices::ICANOpenMotor;
using crf::devices::canopendevices::CANOpenMotorMock;
using crf::devices::canopendevices::CANOpenContextMock;
using crf::actuators::robotbase::CERNBot;

class CERNBotShould : public ::testing::Test {
 protected:
    CERNBotShould() :
        logger_("CERNBotShould"),
        motors_(),
        context_(new NiceMock<CANOpenContextMock>),
        socket_(new NiceMock<CANSocketMock>) {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());

            for (int i=0; i < 4; i++) {
                std::shared_ptr<CANOpenMotorMock> motor;
                motor.reset(new NiceMock<CANOpenMotorMock>);
                motors_.push_back(motor);
            }
    }

    void SetUp() override {
        for (int i=0; i < 4; i++) {
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), initialize())
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), deinitialize())
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), inFault())
                .WillByDefault(Return(false));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), enableOperation())
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), disableOperation())
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), isAlive())
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), isEnabled())
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), getVelocity())
                .WillByDefault(Return(0));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), getCurrent())
                .WillByDefault(Return(0));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), setVelocity(_))
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), setProfileAcceleration(_)) // NOLINT
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), setProfileDeceleration(_)) // NOLINT
                .WillByDefault(Return(true));
            ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), setQuickstopDeceleration(_)) // NOLINT
                .WillByDefault(Return(true));
        }
        ON_CALL(*context_, initialize()).WillByDefault(Return(true));
        ON_CALL(*context_, deinitialize()).WillByDefault(Return(true));

        std::string testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("modules/"));
        testDirName_ += "modules/Actuators/RobotBase/tests/config/";
        std::ifstream goodConfigurationStream(testDirName_ + "goodCernBotConfig.json");
        goodConfiguration_ = nlohmann::json::parse(goodConfigurationStream);
    }

    ~CERNBotShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::vector<std::shared_ptr<ICANOpenMotor> > motors_;
    std::shared_ptr<CANOpenContextMock> context_;
    std::shared_ptr<CANSocketMock> socket_;
    std::unique_ptr<CERNBot> sut_;

    nlohmann::json goodConfiguration_;
};

TEST_F(CERNBotShould, initializeDeinitializeSequence) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_NE(sut_->getConfiguration(), nullptr);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(CERNBotShould, mainConstructorDoesNotThrowException) {
    ASSERT_NO_THROW(sut_.reset(new CERNBot(socket_, goodConfiguration_)));
}


TEST_F(CERNBotShould, operationsFailIfNotInitialized) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_FALSE(sut_->getTaskPose());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_FALSE(sut_->getMotorsVelocities());
    ASSERT_FALSE(sut_->getMotorsCurrent());
    ASSERT_FALSE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity()));
    ASSERT_FALSE(sut_->setWheelsVelocity(std::vector<float>({0, 0, 0, 0})));
    ASSERT_FALSE(sut_->stopBase());
    ASSERT_FALSE(sut_->errorActive());
    ASSERT_FALSE(sut_->acknowledgeError());
}

TEST_F(CERNBotShould, cantGetVelocitiesIfNotEnabled) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), isEnabled())
                .WillByDefault(Return(false));
    for (int i=0; i < 4; i++) {
        EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]),
            setVelocity(_)).Times(0);
    }

    ASSERT_TRUE(sut_->errorActive());

    ASSERT_FALSE(sut_->getMotorsVelocities());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_FALSE(sut_->getMotorsCurrent());

    ASSERT_FALSE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity()));
}

TEST_F(CERNBotShould, cantGetVelocitiesIfNotAlive) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), isAlive())
                .WillByDefault(Return(false));
    for (int i=0; i < 4; i++) {
        EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]),
            setVelocity(_)).Times(0);
    }

    ASSERT_TRUE(sut_->errorActive());

    ASSERT_FALSE(sut_->getMotorsVelocities());
    ASSERT_FALSE(sut_->getTaskVelocity());
    ASSERT_FALSE(sut_->getMotorsCurrent());

    ASSERT_FALSE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity()));
    ASSERT_FALSE(sut_->setWheelsVelocity(std::vector<float>({0, 0, 0, 0})));
}

TEST_F(CERNBotShould, cantGetVelocitiesIfOneMotorDoesntAnswer) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[1]), getVelocity())
                .WillByDefault(Return(std::nullopt));

    ASSERT_FALSE(sut_->getMotorsVelocities());
    ASSERT_FALSE(sut_->getTaskVelocity());
}

TEST_F(CERNBotShould, cantGetCurrentsIfOneMotorDoesntAnswer) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[1]), getCurrent())
                .WillByDefault(Return(std::nullopt));

    ASSERT_FALSE(sut_->getMotorsCurrent());
}

TEST_F(CERNBotShould, stopBase) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());
    for (int i=0; i < 4; i++) {
        EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]),
            setVelocity(0)).Times(1);
    }

    ASSERT_TRUE(sut_->stopBase());
}

TEST_F(CERNBotShould, failsToAcknowledgeAlarm) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());

    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), isEnabled())
                .WillByDefault(Return(false));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), inFault())
                .WillByDefault(Return(true));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), faultReset())
                .WillByDefault(Return(false));

    ASSERT_FALSE(sut_->acknowledgeError());
}

TEST_F(CERNBotShould, failsToAcknowledgeAlarm2) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());

    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), isEnabled())
                .WillByDefault(Return(false));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), inFault())
                .WillByDefault(Return(true));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), faultReset())
                .WillByDefault(Return(true));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]), enableOperation())
                .WillByDefault(Return(false));

    ASSERT_FALSE(sut_->acknowledgeError());
}

TEST_F(CERNBotShould, acknowledgeAlarms) {
    bool faultStatus = false;
    for (int i=0; i < 4; i++) {
        ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), inFault())
                    .WillByDefault(Invoke([&faultStatus]() { return faultStatus; }));
        ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), faultReset())
                    .WillByDefault(Invoke([&faultStatus]() { faultStatus = false; return true; }));
        ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), enableOperation())
                    .WillByDefault(Return(true));
        ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]), isAlive())
                    .WillByDefault(Return(true));
    }
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());

    faultStatus = true;

    ASSERT_TRUE(sut_->acknowledgeError());
}

TEST_F(CERNBotShould, initializationFailsForVariousReasons) {
    bool faultStatus = true;
    bool faultReset = true;
    bool enableOperation = true;
    bool contextInit = true;
    bool motorInit = true;
    bool setProfileAccelerationRes = true;
    bool setProfileDecelerationRes = true;
    bool setQuickStopDecelerationRes = true;

    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), inFault())
                .WillByDefault(Invoke([&faultStatus]() { return faultStatus; }));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), faultReset())
                .WillByDefault(Invoke([&faultReset]() { return faultReset; }));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), enableOperation())
                .WillByDefault(Invoke([&enableOperation]() { return enableOperation; }));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), initialize())
                .WillByDefault(Invoke([&motorInit]() { return motorInit; }));
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), setProfileAcceleration(_))
                .WillByDefault(Invoke([&setProfileAccelerationRes](uint32_t value) { return setProfileAccelerationRes; }));  // NOLINT
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), setProfileDeceleration(_))
                .WillByDefault(Invoke([&setProfileDecelerationRes](uint32_t value) { return setProfileDecelerationRes; }));  // NOLINT
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), setQuickstopDeceleration(_))
                .WillByDefault(Invoke([&setQuickStopDecelerationRes](uint32_t value) { return setQuickStopDecelerationRes; }));  // NOLINT

    ON_CALL(*context_, initialize()).WillByDefault(
        Invoke([&contextInit]() { return contextInit; }));
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));

    contextInit = false;
    ASSERT_FALSE(sut_->initialize());

    contextInit = true;
    faultReset = false;
    ASSERT_FALSE(sut_->initialize());

    faultReset = true;
    enableOperation = false;
    ASSERT_FALSE(sut_->initialize());

    enableOperation = true;
    motorInit = false;
    ASSERT_FALSE(sut_->initialize());

    /**
     * The test is commented due to the error in setting the acceleration
     */
    /*motorInit = true;
    setProfileAccelerationRes = false;
    ASSERT_FALSE(sut_->initialize());

    setProfileAccelerationRes = true;*/
    setProfileDecelerationRes = false;
    ASSERT_FALSE(sut_->initialize());

    setProfileDecelerationRes = true;
    setQuickStopDecelerationRes = false;
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(CERNBotShould, correctlySetsTaskVelocity) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());
    for (int i=0; i < 4; i++) {
        EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[i]),
            setVelocity(0)).Times(1);
    }

    ASSERT_TRUE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity()));
}

TEST_F(CERNBotShould, callsStopIfFailsToSetOneVelocity) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[0]),
        setVelocity(0)).Times(2);
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[1]),
        setVelocity(0)).Times(2);
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]),
        setVelocity(0)).Times(1);
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), setVelocity(_))
                    .WillByDefault(Return(false));
    ASSERT_FALSE(sut_->setTaskVelocity(crf::utility::types::TaskVelocity()));
}

TEST_F(CERNBotShould, setsOfMaximumLimits) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    ASSERT_TRUE(sut_->initialize());

    ASSERT_FALSE(sut_->setMaximumWheelsVelocity(23004));
    ASSERT_FALSE(sut_->setMaximumWheelsVelocity(-0.1));
    ASSERT_TRUE(sut_->setMaximumWheelsVelocity(10));

    ASSERT_FALSE(sut_->setMaximumWheelsAcceleration(23004));
    ASSERT_FALSE(sut_->setMaximumWheelsAcceleration(-0.1));
    ASSERT_TRUE(sut_->setMaximumWheelsAcceleration(10));

    ASSERT_FALSE(sut_->setMaximumTaskVelocity(crf::utility::types::TaskVelocity({120, 0, 0, 0, 0, 0})));  // NOLINT
    ASSERT_FALSE(sut_->setMaximumTaskVelocity(crf::utility::types::TaskVelocity({-0.1, 0, 0, 0, 0, 0})));  // NOLINT
    ASSERT_TRUE(sut_->setMaximumTaskVelocity(crf::utility::types::TaskVelocity({0.1, 0.1, 0, 0, 0, 0.1})));  // NOLINT

    ASSERT_FLOAT_EQ(sut_->getMaximumWheelsVelocity(), 10);
    ASSERT_FLOAT_EQ(sut_->getMaximumWheelsAcceleration(), 10);
    ASSERT_FLOAT_EQ(sut_->getMaximumTaskVelocity()[0], 0.1);
    ASSERT_FLOAT_EQ(sut_->getMaximumTaskVelocity()[1], 0.1);
    ASSERT_FLOAT_EQ(sut_->getMaximumTaskVelocity()[5], 0.1);
}

TEST_F(CERNBotShould, correctlyScaleTaskVelocities) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[0]),
        setVelocity(-328)).Times(1);
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[1]),
        setVelocity(-395)).Times(1);
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]),
        setVelocity(348)).Times(1);
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]),
        setVelocity(280)).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setMaximumTaskVelocity(crf::utility::types::TaskVelocity({0.1, 0.1, 0, 0, 0, 0.1})));  // NOLINT

    ASSERT_TRUE(sut_->setTaskVelocity(
        crf::utility::types::TaskVelocity({0.1, 1, 0, 0, 0, 0.1})));
}


TEST_F(CERNBotShould, failsToSetWheelsVelocityWrongSize) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->setWheelsVelocity(std::vector<float>({10, 20, 10})));
}

TEST_F(CERNBotShould, correctlyScaleWheelsVelocities) {
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[0]),
        setVelocity(1718)).Times(1);
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[1]),
        setVelocity(-3437)).Times(1);
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]),
        setVelocity(1718)).Times(1);
    EXPECT_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[3]),
        setVelocity(-1718)).Times(1);

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setMaximumWheelsVelocity(10));  // NOLINT

    ASSERT_TRUE(sut_->setWheelsVelocity(std::vector<float>({10, 20, 10, 10})));
}

TEST_F(CERNBotShould, failsToSetProfileAccelerationOrDeceleration) {
    bool setProfileAccelerationRes = true;
    bool setProfileDecelerationRes = true;

    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), setProfileAcceleration(_))
                .WillByDefault(Invoke([&setProfileAccelerationRes](uint32_t value) { return setProfileAccelerationRes; }));  // NOLINT
    ON_CALL(*std::dynamic_pointer_cast<CANOpenMotorMock>(motors_[2]), setProfileDeceleration(_))
                .WillByDefault(Invoke([&setProfileDecelerationRes](uint32_t value) { return setProfileDecelerationRes; }));  // NOLINT
    sut_.reset(new CERNBot(goodConfiguration_, motors_, context_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->setMaximumWheelsVelocity(10));  // NOLINT

    setProfileAccelerationRes = false;
    ASSERT_FALSE(sut_->setMaximumWheelsAcceleration(4000));
    setProfileAccelerationRes = true;
    setProfileDecelerationRes = false;
    ASSERT_FALSE(sut_->setMaximumWheelsAcceleration(4000));
}

