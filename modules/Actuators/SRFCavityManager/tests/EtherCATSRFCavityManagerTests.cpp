/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Adrien Luthi CERN EN/SMM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <fstream>

#include "EventLogger/EventLogger.hpp"
#include "EtherCATDevices/EtherCATMotorMock.hpp"

#include "SRFCavityManager/EtherCATSRFCavityManager/EtherCATSRFCavityManager.hpp"

using testing::_;
using testing::AtLeast;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;
using testing::DoDefault;

using crf::devices::ethercatdevices::EtherCATMotorMock;
using crf::devices::ethercatdevices::modesofoperation::ProfilePositionMode;
using crf::devices::ethercatdevices::modesofoperation::ProfileVelocityMode;
using crf::actuators::srfcavityManager::EtherCATSRFCavityManager;


class EtherCATSRFCavityManagerShould : public ::testing::Test {
 protected:
    EtherCATSRFCavityManagerShould() :
        logger_("EtherCATSRFCavityManagerShould"),
        cavityMotor_(new NiceMock<EtherCATMotorMock>) {
            logger_->info("{} BEGIN",
                testing::UnitTest::GetInstance()->current_test_info()->name());
            dirName_ = __FILE__;
            dirName_ = dirName_.substr(0, dirName_.find("tests"));
            dirName_ += "tests/ARIS/config/";
    }

    ~EtherCATSRFCavityManagerShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        unitFactor_ = 333772.107215033;
        encoderPos_ = referenceCavEncoder_;
        encoderResolution_ = 2*M_PI*unitFactor_;
        motorVel_ = 0;
        ON_CALL(*cavityMotor_, initialize()).WillByDefault(Return(true));
        ON_CALL(*cavityMotor_, bindPDOs()).WillByDefault(Return(true));
        ON_CALL(*cavityMotor_, deinitialize()).WillByDefault(Return(true));
        ON_CALL(*cavityMotor_, setMaxCurrent(_)).WillByDefault
            (testing::Invoke([this](uint16_t maxCurrent) {
            return true;
        }));
        ON_CALL(*cavityMotor_, setMaxTorque(_)).WillByDefault
            (testing::Invoke([this](uint16_t maxTorque) {
            return true;
        }));
        ON_CALL(*cavityMotor_, inFault()).WillByDefault(Return(true));
        ON_CALL(*cavityMotor_, faultReset()).WillByDefault(Return(true));
        ON_CALL(*cavityMotor_, shutdown()).WillByDefault(Return(true));
        ON_CALL(*cavityMotor_, enableOperation()).WillByDefault(Return(true));

        ON_CALL(*cavityMotor_, setModeOfOperation(_)).WillByDefault
            (testing::Invoke([this](int8_t modeOpe) {
            if (modeOpe == ProfilePositionMode) {
                modeOfOperation_ = modeOpe;
            } else if (modeOpe == ProfileVelocityMode) {
                modeOfOperation_ = modeOpe;
            } else {
                return false;
            }
            return true;
        }));

        ON_CALL(*cavityMotor_, getModeOfOperation()).WillByDefault(testing::Invoke([this]() {
            if (modeOfOperation_ == ProfilePositionMode) {
                return ProfilePositionMode;
            } else {
                return ProfileVelocityMode;
            }
        }));

        ON_CALL(*cavityMotor_, getPosition()).WillByDefault(testing::Invoke([this]() {
            return encoderPos_;
        }));

        ON_CALL(*cavityMotor_, setPosition(_, _, _, _, _)).WillByDefault(testing::Invoke([this](
            int32_t pos, uint32_t vel, uint32_t acc, uint32_t dec, bool relative) {
                if (!relative) {
                    encoderPos_ = pos;
                } else {
                    encoderPos_ += pos;
                }
            return true;
        }));

        ON_CALL(*cavityMotor_, setVelocity(_, _, _)).WillByDefault(testing::Invoke([this](
            uint32_t vel, uint32_t acc, uint32_t dec) {
                motorVel_ = -vel;
            return true;
        }));

        ON_CALL(*cavityMotor_, targetReached()).WillByDefault(Return(true));

        ON_CALL(*cavityMotor_, getVelocity()).WillByDefault(testing::Invoke([this]() {
            return -motorVel_;
        }));
    }

    crf::utility::logger::EventLogger logger_;
    nlohmann::json configurationJson_;
    std::unique_ptr<EtherCATSRFCavityManager> cavityUt_;
    std::shared_ptr<NiceMock<crf::devices::ethercatdevices::EtherCATMotorMock>> cavityMotor_;

    double encoderPos_;
    double unitFactor_;
    double encoderResolution_;
    double offset_ = 0;
    double velocityTarget_;
    double motorVel_;
    const int referenceCavEncoder_ = 466009;  // found once from calibration
    std::string dirName_;
    int8_t modeOfOperation_;
};

TEST_F(EtherCATSRFCavityManagerShould, testMissingParameterInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigEmpty.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::runtime_error);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongUnitFactorInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongUnitfactor.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongVelocityProfileInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongVelocityProfile.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongAccelerationProfileInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongAccelererationProfile.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongDecelerationProfileInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongDecelererationProfile.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongReferenceCavEncoderInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongReferenceCavEncoder.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongMaxCurrentInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongMaxCurrent.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongMaxTorqueInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongMaxTorque.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongOrientationInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongOrientation.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongPosCommandTimeoutInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongPositionCommandTimeoutSeconds.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongVelocityCommandTimeoutSecondsInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongVelocityCommandTimeoutSeconds.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongStopCommandTimeoutSecondsInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongStopCommandTimeoutSeconds.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, testWrongUpdateIntervalMillisecondsInJsonConfigFile) {
    std::ifstream robotData(dirName_ + "ARISConfigWrongUpdateIntervalMilliseconds.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    EXPECT_THROW(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_),
        std::invalid_argument);
}

TEST_F(EtherCATSRFCavityManagerShould, initializeDeinitializeSequence) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_FALSE(cavityUt_->deinitialize());
    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_FALSE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->deinitialize());
    ASSERT_FALSE(cavityUt_->deinitialize());

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_FALSE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->deinitialize());
    (cavityUt_->deinitialize());
}


TEST_F(EtherCATSRFCavityManagerShould, tryToEnabledMotorNotInitialized) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));
    offset_ = 50*M_PI/180;
    ASSERT_FALSE(cavityUt_->enableMotor());
    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_FALSE(cavityUt_->setPosition(offset_));
    ASSERT_TRUE(cavityUt_->enableMotor());
    ASSERT_TRUE(cavityUt_->enableMotor());
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
}

TEST_F(EtherCATSRFCavityManagerShould, usingMotorNotEnabled) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->getPosition());
    ASSERT_FALSE(cavityUt_->getVelocity());
    offset_ = 50*M_PI/180;
    ASSERT_FALSE(cavityUt_->setPosition(offset_));
    ASSERT_FALSE(cavityUt_->setVelocity(5*M_PI/180));
    ASSERT_FALSE(cavityUt_->stop());
    ASSERT_FALSE(cavityUt_->isTurning());
}

TEST_F(EtherCATSRFCavityManagerShould, moveToOriginFailedIfNotIn0AndNotEnabled) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    encoderPos_ = 1;
    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_FALSE(cavityUt_->moveToOrigin());
}

TEST_F(EtherCATSRFCavityManagerShould, moveToOriginSuccessIfIn0AndNotEnabled) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->moveToOrigin());
}

TEST_F(EtherCATSRFCavityManagerShould, setGetOrientationSequence) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));
    ASSERT_TRUE(cavityUt_->initialize());

    CavityOrientation orientation = CavityOrientation::IO;
    cavityUt_->setCavityOrientation(orientation);
    EXPECT_EQ(orientation, cavityUt_->getCavityOrientation());
}

TEST_F(EtherCATSRFCavityManagerShould, getPositionWithoutMotorInitialization) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));
    ASSERT_FALSE(cavityUt_->getPosition());
}

TEST_F(EtherCATSRFCavityManagerShould, setNotRelativeAndRelativePositionInRange0_360) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    offset_ = 50*M_PI/180;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    // Go to 50deg from origin (relative) => expects 50
    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);

    // Go to 50deg more => expects 100
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), 2*offset_, 0.01);

    // Go to -50deg less => expects 50
    ASSERT_TRUE(cavityUt_->setPosition(-offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);

    // Go to 150deg (not relative) => expects 150
    ASSERT_TRUE(cavityUt_->setPosition(3*offset_, true));
    EXPECT_NEAR(cavityUt_->getPosition().value(), 3*offset_, 0.01);
}

TEST_F(EtherCATSRFCavityManagerShould, setPositionTo2PI) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    offset_ = 2*M_PI;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), 0, 0.01);

    offset_ = -2*M_PI;
    ASSERT_TRUE(cavityUt_->setPosition(offset_, true));
    EXPECT_NEAR(cavityUt_->getPosition().value(), 0, 0.01);
}

TEST_F(EtherCATSRFCavityManagerShould, setPositionToZeroWhileBeingAtZero) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    offset_ = 0;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);

    ASSERT_TRUE(cavityUt_->setPosition(offset_, true));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);
}

TEST_F(EtherCATSRFCavityManagerShould, goToOriginSequence) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    offset_ = 90*M_PI/180;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());

    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    ASSERT_TRUE(cavityUt_->moveToOrigin());

    offset_ = 170*M_PI/180;
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    ASSERT_TRUE(cavityUt_->moveToOrigin());

    offset_ = 270*M_PI/180;
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    ASSERT_TRUE(cavityUt_->moveToOrigin());
}

TEST_F(EtherCATSRFCavityManagerShould, testSetPosition) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    offset_ = 50*M_PI/180;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);

    ASSERT_TRUE(cavityUt_->setPosition(0));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);

    ASSERT_TRUE(cavityUt_->setPosition(0, true));
    EXPECT_NEAR(cavityUt_->getPosition().value(), 0, 0.01);

    offset_ = 400*M_PI/180;
    ASSERT_FALSE(cavityUt_->setPosition(offset_));

    offset_ = -400*M_PI/180;
    ASSERT_FALSE(cavityUt_->setPosition(offset_));

    offset_ = -100*M_PI/180;
    ASSERT_TRUE(cavityUt_->setPosition(offset_, true));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_+2*M_PI, 0.01);

    offset_ = 200*M_PI/180;
    ASSERT_TRUE(cavityUt_->setPosition(offset_, true));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);
}

TEST_F(EtherCATSRFCavityManagerShould, setPositionToOriginWhileBeingAtZero) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    offset_ = 0;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);

    offset_ = 50*M_PI/180;
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_, 0.01);
    ASSERT_TRUE(cavityUt_->moveToOrigin());
    EXPECT_NEAR(cavityUt_->getPosition().value(), 0, 0.01);

    offset_ = -50*M_PI/180;
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
    EXPECT_NEAR(cavityUt_->getPosition().value(), offset_+2*M_PI, 0.01);
    ASSERT_TRUE(cavityUt_->moveToOrigin());

    offset_ = 5000*M_PI/180;
    ASSERT_FALSE(cavityUt_->setPosition(offset_));

    offset_ = -5000*M_PI/180;
    ASSERT_FALSE(cavityUt_->setPosition(offset_));
}

TEST_F(EtherCATSRFCavityManagerShould, setPositionWhileModeOpeIsVelocity) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    offset_ = 180*M_PI/180;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    cavityMotor_->setModeOfOperation(ProfileVelocityMode);
    ASSERT_TRUE(cavityUt_->setPosition(offset_));
}

TEST_F(EtherCATSRFCavityManagerShould, setVelocity) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    velocityTarget_ = 5*M_PI/180;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    cavityMotor_->setModeOfOperation(ProfileVelocityMode);
    ASSERT_TRUE(cavityUt_->setVelocity(velocityTarget_));
}

TEST_F(EtherCATSRFCavityManagerShould, setVelocityOutOfUpperRange) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    velocityTarget_ = 20*M_PI/180;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    cavityMotor_->setModeOfOperation(ProfileVelocityMode);
    ASSERT_FALSE(cavityUt_->setVelocity(velocityTarget_));
}

TEST_F(EtherCATSRFCavityManagerShould, setVelocityOutOfLowerRange) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    velocityTarget_ = -20*M_PI/180;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    cavityMotor_->setModeOfOperation(ProfileVelocityMode);
    ASSERT_FALSE(cavityUt_->setVelocity(velocityTarget_));
}

TEST_F(EtherCATSRFCavityManagerShould, setVelocityWhileModeOpeIsPosition) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    velocityTarget_ = 5*M_PI/180;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->initialize());
    ASSERT_TRUE(cavityUt_->enableMotor());
    cavityMotor_->setModeOfOperation(ProfilePositionMode);
    ASSERT_TRUE(cavityUt_->setVelocity(velocityTarget_));
}

TEST_F(EtherCATSRFCavityManagerShould, checkingSetCavityTypeReability) {
    std::ifstream robotData(dirName_ + "ARISConfig.json");
    nlohmann::json configurationJson_ = nlohmann::json::parse(robotData);
    float laserValue = 1300.523;
    cavityUt_.reset(new EtherCATSRFCavityManager(configurationJson_, cavityMotor_));

    ASSERT_TRUE(cavityUt_->setCavityType(laserValue));
    EXPECT_EQ(cavityUt_->getCavityType(), CavityType::FiveCells);

    laserValue = 1100.523;
    ASSERT_TRUE(cavityUt_->setCavityType(laserValue));
    EXPECT_EQ(cavityUt_->getCavityType(), CavityType::LHC);

    laserValue = 400.523;
    ASSERT_TRUE(cavityUt_->setCavityType(laserValue));
    EXPECT_EQ(cavityUt_->getCavityType(), CavityType::FCC);

    laserValue = -500.523;
    ASSERT_FALSE(cavityUt_->setCavityType(laserValue));
    EXPECT_EQ(cavityUt_->getCavityType(), CavityType::Undefined);

    laserValue = 50000.523;
    ASSERT_FALSE(cavityUt_->setCavityType(laserValue));
    EXPECT_EQ(cavityUt_->getCavityType(), CavityType::Undefined);

    laserValue = 0;
    ASSERT_FALSE(cavityUt_->setCavityType(laserValue));
    EXPECT_EQ(cavityUt_->getCavityType(), CavityType::Undefined);
}

