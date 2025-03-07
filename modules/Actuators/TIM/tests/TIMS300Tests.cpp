/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <fstream>

#include <chrono>
#include <thread>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <Snap7/s7.hpp>

#include "EventLogger/EventLogger.hpp"
#include "SiemensPLC/SiemensPLCMock.hpp"
#include "SiemensPLC/RegisterType.hpp"
#include "TIM/TIMAlarms.hpp"
#include "TIM/TIMCommands.hpp"
#include "TIM/TIMSettings.hpp"
#include "TIM/TIMStatus.hpp"
#include "TIM/TIMConfiguration.hpp"
#include "TIM/TIMS300/TIMS300.hpp"

using testing::_;

class TIMS300Should : public ::testing::Test {
 protected:
    TIMS300Should() :
        logger_("TIMS300Should"),
        isPLCConnected_(false),
        mockAlarms_(),
        mockCommands_(),
        mockSettings_(),
        mockStatus_() {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());

        testDirName_ = __FILE__;
        testDirName_ = testDirName_.substr(0, testDirName_.find("tests"));
        testDirName_ += "tests/config/";

        mockAlarms_.barcodeReaderError(false);
        mockAlarms_.batteryError(false);
        mockAlarms_.chargingArmMotorError(false);
        mockAlarms_.chargingArmRequiresAcknowledgement(false);
        mockAlarms_.frontBumperPressed(false);
        mockAlarms_.frontLaserScannerError(false);
        mockAlarms_.frontProtectiveFieldReading(false);
        mockAlarms_.backBumperPressed(false);
        mockAlarms_.backLaserScannerError(false);
        mockAlarms_.backProtectiveFieldReading(false);
        mockAlarms_.mainMotorError(false);
        mockAlarms_.mainMotorRequiresAcknowledgement(false);
        mockAlarms_.positionEncoderError(false);
        mockAlarms_.positionEncoderReadingError(false);
        mockAlarms_.velocityEncoderError(false);
        mockAlarms_.velocityEncoderReadingError(false);
        mockAlarms_.emergencyStop(false);

        mockCommands_.localHeartbeat(0);
        mockCommands_.setPositionManually(false);
        mockCommands_.moveToTarget(false);
        mockCommands_.jogForward(false);
        mockCommands_.jogBackward(false);
        mockCommands_.stop(false);
        mockCommands_.emergencyStop(false);
        mockCommands_.chargingArmManualControl(false);
        mockCommands_.startCharging(false);
        mockCommands_.stopCharging(false);
        mockCommands_.extendChargingArm(false);
        mockCommands_.retractChargingArm(false);
        mockCommands_.enableEconomyMode(false);
        mockCommands_.disableEconomyMode(false);
        mockCommands_.rebootRobotArmWagon(false);
        mockCommands_.setObstacleMaximumVelocity(false);
        mockCommands_.devicesRetracted(false);
        mockCommands_.allowMovement(false);
        mockCommands_.acknowledgeAlarms(false);

        mockSettings_.targetPosition(0);
        mockSettings_.targetVelocity(0);
        mockSettings_.positionSetManually(0);
        mockSettings_.obstacleID(0);
        mockSettings_.obstacleMaximumVelocity(0);

        mockStatus_.timPosition(0.25);
        mockStatus_.timVelocity(1);
        mockStatus_.targetReached(false);
        mockStatus_.timStopped(false);
        mockStatus_.charging(false);
        mockStatus_.chargingCurrent(0);
        mockStatus_.batteryVoltage(0);
        mockStatus_.batteryCurrent(0);
        mockStatus_.economyMode(false);
        mockStatus_.chargingArmConnected(false);
        mockStatus_.chargingArmRetracted(false);
        mockStatus_.frontWarningField(false);
        mockStatus_.backWarningField(false);
        mockStatus_.timHeartbeat(0);
        mockStatus_.mainMotorOn(false);
        mockStatus_.safeToMove(false);

        plc_.reset(new testing::NiceMock<crf::devices::siemensplc::SiemensPLCMock>);
    }

    ~TIMS300Should() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        ON_CALL(*plc_, initialize()).WillByDefault(testing::Invoke([this]() {
            isPLCConnected_ = true;
            return true;
        }));
        ON_CALL(*plc_, deinitialize()).WillByDefault(testing::Invoke([this]() {
            isPLCConnected_ = false;
            return true;
        }));
        ON_CALL(*plc_, isConnected()).WillByDefault(testing::Invoke([this]() {
            return isPLCConnected_;
        }));
        ON_CALL(*plc_, writeRegister(_, _, _, _, _)).WillByDefault(testing::Invoke([this](
            crf::devices::siemensplc::RegisterType registerType, const boost::any& value,
            unsigned int dbNumber, unsigned int registerOffset, unsigned int bitNumber) {
            return isPLCConnected_;
        }));
        ON_CALL(*plc_, readDB(_, _, _)).WillByDefault(testing::Invoke([this](
            const unsigned int dbNumber, const size_t length, const unsigned int registerOffset) {
            if (mockStatus_.timHeartbeat() == maxHeartbeatValue) {
                mockStatus_.timHeartbeat(0);
            }
            mockStatus_.timHeartbeat(mockStatus_.timHeartbeat()+1);
            if (dbNumber == datablockNumber4) {
                return serialize4Datablock();
            } else if (dbNumber == datablockNumber509) {
                return serialize509Datablock();
            } else if (dbNumber == datablockNumber510) {
                return serialize510Datablock();
            } else if (dbNumber == datablockNumber511) {
                return serialize511Datablock();
            }
            return std::string();
        }));
    }

    std::string serialize4Datablock() {
        unsigned char buffer[datablockLength4];

        std::map<std::string, std::array<unsigned int, 2>> obstaclesVariablesLocation =
            tim_->getConfiguration()->getObstacleVariablesLocation();

        S7_SetRealAt(buffer, obstaclesVariablesLocation["StartPosition"][0],
            obsStartPosition1_);
        S7_SetRealAt(buffer, obstaclesVariablesLocation["EndPosition"][0],
            obsEndPosition1_);
        S7_SetRealAt(buffer, obstaclesVariablesLocation["MaximumVelocity"][0],
            obsMaximumVelocity1_);
        S7_SetBitAt(buffer, obstaclesVariablesLocation["MustRetractDevices"][0],
            obstaclesVariablesLocation["MustRetractDevices"][1],
            obsMustRetractDevices1_);
        S7_SetBitAt(buffer, obstaclesVariablesLocation["Enabled"][0],
            obstaclesVariablesLocation["Enabled"][1],
            obsEnabled1_);

        return std::string(reinterpret_cast<char*>(buffer), datablockLength4);
    }

    std::string serialize509Datablock() {
        unsigned char buffer[datablockLength509];

        std::map<std::string, std::array<unsigned int, 2>> commandsVariablesLocation =
            tim_->getConfiguration()->getCommandsVariablesLocation();

        S7_SetIntAt(buffer, commandsVariablesLocation["LocalHeartbeat"][0],
            mockCommands_.localHeartbeat());
        S7_SetBitAt(buffer, commandsVariablesLocation["SetPositionManually"][0],
            commandsVariablesLocation["SetPositionManually"][1],
            mockCommands_.setPositionManually());
        S7_SetBitAt(buffer, commandsVariablesLocation["MoveToTarget"][0],
            commandsVariablesLocation["MoveToTarget"][1],
            mockCommands_.moveToTarget());
        S7_SetBitAt(buffer, commandsVariablesLocation["JogForward"][0],
            commandsVariablesLocation["JogForward"][1],
            mockCommands_.jogForward());
        S7_SetBitAt(buffer, commandsVariablesLocation["JogBackward"][0],
            commandsVariablesLocation["JogBackward"][1],
            mockCommands_.jogBackward());
        S7_SetBitAt(buffer, commandsVariablesLocation["Stop"][0],
            commandsVariablesLocation["Stop"][1],
            mockCommands_.stop());
        S7_SetBitAt(buffer, commandsVariablesLocation["EmergencyStop"][0],
            commandsVariablesLocation["EmergencyStop"][1],
            mockCommands_.emergencyStop());
        S7_SetBitAt(buffer, commandsVariablesLocation["ChargingArmManualControl"][0],
            commandsVariablesLocation["ChargingArmManualControl"][1],
            mockCommands_.chargingArmManualControl());
        S7_SetBitAt(buffer, commandsVariablesLocation["StartCharging"][0],
            commandsVariablesLocation["StartCharging"][1],
            mockCommands_.startCharging());
        S7_SetBitAt(buffer, commandsVariablesLocation["StopCharging"][0],
            commandsVariablesLocation["StopCharging"][1],
            mockCommands_.stopCharging());
        S7_SetBitAt(buffer, commandsVariablesLocation["ExtendChargingArm"][0],
            commandsVariablesLocation["ExtendChargingArm"][1],
            mockCommands_.extendChargingArm());
        S7_SetBitAt(buffer, commandsVariablesLocation["RetractChargingArm"][0],
            commandsVariablesLocation["RetractChargingArm"][1],
            mockCommands_.retractChargingArm());
        S7_SetBitAt(buffer, commandsVariablesLocation["EnableEconomyMode"][0],
            commandsVariablesLocation["EnableEconomyMode"][1],
            mockCommands_.enableEconomyMode());
        S7_SetBitAt(buffer, commandsVariablesLocation["DisableEconomyMode"][0],
            commandsVariablesLocation["DisableEconomyMode"][1],
            mockCommands_.disableEconomyMode());
        S7_SetBitAt(buffer, commandsVariablesLocation["RebootRobotArmWagon"][0],
            commandsVariablesLocation["RebootRobotArmWagon"][1],
            mockCommands_.rebootRobotArmWagon());
        S7_SetBitAt(buffer, commandsVariablesLocation["SetObstacleMaximumVelocity"][0],
            commandsVariablesLocation["SetObstacleMaximumVelocity"][1],
            mockCommands_.setObstacleMaximumVelocity());
        S7_SetBitAt(buffer, commandsVariablesLocation["DevicesRetracted"][0],
            commandsVariablesLocation["DevicesRetracted"][1],
            mockCommands_.devicesRetracted());
        S7_SetBitAt(buffer, commandsVariablesLocation["AllowMovement"][0],
            commandsVariablesLocation["AllowMovement"][1],
            mockCommands_.allowMovement());
        S7_SetBitAt(buffer, commandsVariablesLocation["AcknowledgeAlarms"][0],
            commandsVariablesLocation["AcknowledgeAlarms"][1],
            mockCommands_.acknowledgeAlarms());

        return std::string(reinterpret_cast<char*>(buffer), datablockLength509);
    }

    std::string serialize510Datablock() {
        unsigned char buffer[datablockLength510];

        std::map<std::string, std::array<unsigned int, 2>> settingsVariablesLocation =
            tim_->getConfiguration()->getSettingsVariablesLocation();

        S7_SetRealAt(buffer, settingsVariablesLocation["TargetPosition"][0],
            mockSettings_.targetPosition());
        S7_SetRealAt(buffer, settingsVariablesLocation["TargetVelocity"][0],
            mockSettings_.targetVelocity());
        S7_SetRealAt(buffer, settingsVariablesLocation["PositionSetManually"][0],
            mockSettings_.positionSetManually());
        S7_SetSIntAt(buffer, settingsVariablesLocation["ObstacleID"][0],
            mockSettings_.obstacleID());
        S7_SetRealAt(buffer, settingsVariablesLocation["ObstacleMaximumVelocity"][0],
            mockSettings_.obstacleMaximumVelocity());

        return std::string(reinterpret_cast<char*>(buffer), datablockLength510);
    }

    std::string serialize511Datablock() {
        unsigned char buffer[datablockLength511];

        std::map<std::string, std::array<unsigned int, 2>> statusVariablesLocation =
            tim_->getConfiguration()->getStatusVariablesLocation();

        S7_SetRealAt(buffer, statusVariablesLocation["TIMPosition"][0],
            mockStatus_.timPosition());
        S7_SetRealAt(buffer, statusVariablesLocation["TIMVelocity"][0],
            mockStatus_.timVelocity());
        S7_SetBitAt(buffer, statusVariablesLocation["TargetReached"][0],
            statusVariablesLocation["TargetReached"][1],
            mockStatus_.targetReached());
        S7_SetBitAt(buffer, statusVariablesLocation["TIMStopped"][0],
            statusVariablesLocation["TIMStopped"][1],
            mockStatus_.timStopped());
        S7_SetBitAt(buffer, statusVariablesLocation["Charging"][0],
            statusVariablesLocation["Charging"][1],
            mockStatus_.charging());
        S7_SetRealAt(buffer, statusVariablesLocation["ChargingCurrent"][0],
            mockStatus_.chargingCurrent());
        S7_SetRealAt(buffer, statusVariablesLocation["BatteryVoltage"][0],
            mockStatus_.batteryVoltage());
        S7_SetRealAt(buffer, statusVariablesLocation["BatteryCurrent"][0],
            mockStatus_.batteryCurrent());
        S7_SetBitAt(buffer, statusVariablesLocation["EconomyMode"][0],
            statusVariablesLocation["EconomyMode"][1],
            mockStatus_.economyMode());
        S7_SetBitAt(buffer, statusVariablesLocation["ChargingArmConnected"][0],
            statusVariablesLocation["ChargingArmConnected"][1],
            mockStatus_.chargingArmConnected());
        S7_SetBitAt(buffer, statusVariablesLocation["ChargingArmRetracted"][0],
            statusVariablesLocation["ChargingArmRetracted"][1],
            mockStatus_.chargingArmRetracted());
        S7_SetBitAt(buffer, statusVariablesLocation["FrontWarningField"][0],
            statusVariablesLocation["FrontWarningField"][1],
            mockStatus_.frontWarningField());
        S7_SetBitAt(buffer, statusVariablesLocation["BackWarningField"][0],
            statusVariablesLocation["BackWarningField"][1],
            mockStatus_.backWarningField());
        S7_SetIntAt(buffer, statusVariablesLocation["TIMHeartbeat"][0],
            mockStatus_.timHeartbeat());
        S7_SetBitAt(buffer, statusVariablesLocation["MainMotorOn"][0],
            statusVariablesLocation["MainMotorOn"][1],
            mockStatus_.mainMotorOn());
        S7_SetBitAt(buffer, statusVariablesLocation["SafeToMove"][0],
            statusVariablesLocation["SafeToMove"][1],
            mockStatus_.safeToMove());

        std::map<std::string, std::array<unsigned int, 2>> alarmVariablesLocation =
            tim_->getConfiguration()->getAlarmVariablesLocation();

        S7_SetBitAt(buffer, alarmVariablesLocation["BarcodeReaderError"][0],
            alarmVariablesLocation["BarcodeReaderError"][1],
            mockAlarms_.barcodeReaderError());
        S7_SetBitAt(buffer, alarmVariablesLocation["BatteryError"][0],
            alarmVariablesLocation["BatteryError"][1],
            mockAlarms_.batteryError());
        S7_SetBitAt(buffer, alarmVariablesLocation["ChargingArmMotorError"][0],
            alarmVariablesLocation["ChargingArmMotorError"][1],
            mockAlarms_.chargingArmMotorError());
        S7_SetBitAt(buffer, alarmVariablesLocation["ChargingArmRequiresAcknowledgement"][0],
            alarmVariablesLocation["ChargingArmRequiresAcknowledgement"][1],
            mockAlarms_.chargingArmRequiresAcknowledgement());
        S7_SetBitAt(buffer, alarmVariablesLocation["FrontBumperPressed"][0],
            alarmVariablesLocation["FrontBumperPressed"][1],
            mockAlarms_.frontBumperPressed());
        S7_SetBitAt(buffer, alarmVariablesLocation["FrontLaserScannerError"][0],
            alarmVariablesLocation["FrontLaserScannerError"][1],
            mockAlarms_.frontLaserScannerError());
        S7_SetBitAt(buffer, alarmVariablesLocation["FrontProtectiveFieldReading"][0],
            alarmVariablesLocation["FrontProtectiveFieldReading"][1],
            mockAlarms_.frontProtectiveFieldReading());
        S7_SetBitAt(buffer, alarmVariablesLocation["BackBumperPressed"][0],
            alarmVariablesLocation["BackBumperPressed"][1],
            mockAlarms_.backBumperPressed());
        S7_SetBitAt(buffer, alarmVariablesLocation["BackLaserScannerError"][0],
            alarmVariablesLocation["BackLaserScannerError"][1],
            mockAlarms_.backLaserScannerError());
        S7_SetBitAt(buffer, alarmVariablesLocation["BackProtectiveFieldReading"][0],
            alarmVariablesLocation["BackProtectiveFieldReading"][1],
            mockAlarms_.backProtectiveFieldReading());
        S7_SetBitAt(buffer, alarmVariablesLocation["MainMotorError"][0],
            alarmVariablesLocation["MainMotorError"][1],
            mockAlarms_.mainMotorError());
        S7_SetBitAt(buffer, alarmVariablesLocation["MainMotorRequiresAcknowledgement"][0],
            alarmVariablesLocation["MainMotorRequiresAcknowledgement"][1],
            mockAlarms_.mainMotorRequiresAcknowledgement());
        S7_SetBitAt(buffer, alarmVariablesLocation["PositionEncoderError"][0],
            alarmVariablesLocation["PositionEncoderError"][1],
            mockAlarms_.positionEncoderError());
        S7_SetBitAt(buffer, alarmVariablesLocation["PositionEncoderReadingError"][0],
            alarmVariablesLocation["PositionEncoderReadingError"][1],
            mockAlarms_.positionEncoderReadingError());
        S7_SetBitAt(buffer, alarmVariablesLocation["VelocityEncoderError"][0],
            alarmVariablesLocation["VelocityEncoderError"][1],
            mockAlarms_.velocityEncoderError());
        S7_SetBitAt(buffer, alarmVariablesLocation["VelocityEncoderReadingError"][0],
            alarmVariablesLocation["VelocityEncoderReadingError"][1],
            mockAlarms_.velocityEncoderReadingError());
        S7_SetBitAt(buffer, alarmVariablesLocation["EmergencyStop"][0],
            alarmVariablesLocation["EmergencyStop"][1],
            mockAlarms_.emergencyStop());

        return std::string(reinterpret_cast<char*>(buffer), datablockLength511);
    }

    void getGoodTIMValues() {
        std::ifstream timData(testDirName_ + "timGoodConfig.json");
        nlohmann::json timJSON = nlohmann::json::parse(timData);

        ASSERT_TRUE(tim_->isConnected());

        ASSERT_TRUE(tim_->getCurrentPosition());
        ASSERT_EQ(tim_->getCurrentPosition().value(),
            mockStatus_.timPosition() + timJSON["PositionOffset"].get<float>());
        ASSERT_TRUE(tim_->getTargetPosition());
        ASSERT_EQ(tim_->getTargetPosition().value(),
            mockSettings_.targetPosition() + timJSON["PositionOffset"].get<float>());
        ASSERT_TRUE(tim_->getCurrentVelocity());
        ASSERT_EQ(tim_->getCurrentVelocity().value(), mockStatus_.timVelocity());
        ASSERT_TRUE(tim_->getTargetVelocity());
        ASSERT_EQ(tim_->getTargetVelocity().value(), mockSettings_.targetVelocity());
        ASSERT_TRUE(tim_->getCurrentMaximumVelocity());
        ASSERT_EQ(tim_->getCurrentMaximumVelocity().value(),
            timJSON["Limits"]["MaximumVelocity"].get<float>());
        ASSERT_TRUE(tim_->isMoving());
        ASSERT_EQ(tim_->isMoving().value(), !mockStatus_.timStopped());
        ASSERT_TRUE(tim_->isTargetReached());
        ASSERT_EQ(tim_->isTargetReached().value(), mockStatus_.targetReached());

        ASSERT_TRUE(tim_->isCharging());
        ASSERT_EQ(tim_->isCharging().value(), mockStatus_.charging());
        ASSERT_TRUE(tim_->getChargingCurrent());
        ASSERT_EQ(tim_->getChargingCurrent().value(), mockStatus_.chargingCurrent());
        ASSERT_TRUE(tim_->getBatteryVoltage());
        ASSERT_EQ(tim_->getBatteryVoltage().value(), mockStatus_.batteryVoltage());
        ASSERT_TRUE(tim_->getBatteryCurrent());
        ASSERT_EQ(tim_->getBatteryCurrent().value(), mockStatus_.batteryCurrent());
        ASSERT_TRUE(tim_->isInEconomy());
        ASSERT_EQ(tim_->isInEconomy().value(), mockStatus_.economyMode());

        // std::map<int, std::tuple<float, float, float>> getClosestObstacleAreas()
        // std::optional<bool> isInObstacleArea()
        // std::optional<bool> devicesRetracted()
        ASSERT_TRUE(tim_->isFrontWarningFieldActive());
        ASSERT_EQ(tim_->isFrontWarningFieldActive().value(), mockStatus_.frontWarningField());
        ASSERT_TRUE(tim_->isBackWarningFieldActive());
        ASSERT_EQ(tim_->isBackWarningFieldActive().value(), mockStatus_.backWarningField());
        ASSERT_EQ(tim_->isSafeToMove().value(), mockStatus_.safeToMove());

        crf::actuators::tim::TIMAlarms alarms = tim_->getAlarms();
        ASSERT_FALSE(alarms.isEmpty());
        ASSERT_EQ(alarms.barcodeReaderError(), mockAlarms_.barcodeReaderError());
        ASSERT_EQ(alarms.batteryError(), mockAlarms_.batteryError());
        ASSERT_EQ(alarms.chargingArmMotorError(), mockAlarms_.chargingArmMotorError());
        ASSERT_EQ(alarms.chargingArmRequiresAcknowledgement(),
            mockAlarms_.chargingArmRequiresAcknowledgement());
        ASSERT_EQ(alarms.frontBumperPressed(), mockAlarms_.frontBumperPressed());
        ASSERT_EQ(alarms.frontLaserScannerError(), mockAlarms_.frontLaserScannerError());
        ASSERT_EQ(alarms.frontProtectiveFieldReading(), mockAlarms_.frontProtectiveFieldReading());
        ASSERT_EQ(alarms.backBumperPressed(), mockAlarms_.backBumperPressed());
        ASSERT_EQ(alarms.backLaserScannerError(), mockAlarms_.backLaserScannerError());
        ASSERT_EQ(alarms.backProtectiveFieldReading(), mockAlarms_.backProtectiveFieldReading());
        ASSERT_EQ(alarms.mainMotorError(), mockAlarms_.mainMotorError());
        ASSERT_EQ(alarms.mainMotorRequiresAcknowledgement(),
            mockAlarms_.mainMotorRequiresAcknowledgement());
        ASSERT_EQ(alarms.positionEncoderError(), mockAlarms_.positionEncoderError());
        ASSERT_EQ(alarms.positionEncoderReadingError(), mockAlarms_.positionEncoderReadingError());
        ASSERT_EQ(alarms.velocityEncoderError(), mockAlarms_.velocityEncoderError());
        ASSERT_EQ(alarms.velocityEncoderReadingError(), mockAlarms_.velocityEncoderReadingError());
        ASSERT_EQ(alarms.emergencyStop(), mockAlarms_.emergencyStop());
    }

    void setGoodTIMValues() {
        ASSERT_TRUE(tim_->setCurrentPosition(4500));
        ASSERT_TRUE(tim_->setTargetPosition(4000));
        ASSERT_TRUE(tim_->setTargetVelocity(1.0));
        mockStatus_.timStopped(false);
        mockStatus_.safeToMove(true);
        ASSERT_TRUE(tim_->moveToTarget(3200, 0.3));
        ASSERT_TRUE(tim_->jog(0.1));
        mockStatus_.timStopped(true);
        ASSERT_TRUE(tim_->stop());
        mockStatus_.charging(true);
        mockStatus_.chargingArmConnected(true);
        ASSERT_TRUE(tim_->startCharging());
        mockStatus_.charging(false);
        mockStatus_.chargingArmConnected(false);
        ASSERT_TRUE(tim_->stopCharging());
        mockStatus_.economyMode(true);
        ASSERT_TRUE(tim_->enableEconomyMode());
        mockStatus_.economyMode(false);
        ASSERT_TRUE(tim_->disableEconomyMode());
        ASSERT_TRUE(tim_->acknowledgeAlarms());
    }

    void getBadTIMValues() {
        // bool isConnected()
        ASSERT_FALSE(tim_->getCurrentPosition());
        ASSERT_FALSE(tim_->getTargetPosition());
        ASSERT_FALSE(tim_->getCurrentVelocity());
        ASSERT_FALSE(tim_->getTargetVelocity());
        // std::optional<float> getCurrentMaximumVelocity()
        ASSERT_FALSE(tim_->isMoving());
        ASSERT_FALSE(tim_->isTargetReached());
        ASSERT_FALSE(tim_->isCharging());
        ASSERT_FALSE(tim_->getChargingCurrent());
        ASSERT_FALSE(tim_->getBatteryVoltage());
        ASSERT_FALSE(tim_->getBatteryCurrent());
        ASSERT_FALSE(tim_->isInEconomy());
        // std::map<int, std::tuple<float, float, float>> getClosestObstacleAreas()
        // std::optional<bool> isInObstacleArea()
        // std::optional<bool> devicesRetracted()
        ASSERT_FALSE(tim_->isFrontWarningFieldActive());
        ASSERT_FALSE(tim_->isBackWarningFieldActive());
        ASSERT_FALSE(tim_->isSafeToMove());

        crf::actuators::tim::TIMAlarms alarms = tim_->getAlarms();
        ASSERT_TRUE(alarms.isEmpty());
    }

    void setBadTIMValues() {
        ASSERT_FALSE(tim_->setCurrentPosition(4500));
        ASSERT_FALSE(tim_->setTargetPosition(4000));
        ASSERT_FALSE(tim_->setTargetVelocity(1.0));
        ASSERT_FALSE(tim_->moveToTarget(3200, 0.3));
        ASSERT_FALSE(tim_->jog(0.1));
        ASSERT_FALSE(tim_->stop());
        ASSERT_FALSE(tim_->emergencyStop());
        ASSERT_FALSE(tim_->startCharging());
        ASSERT_FALSE(tim_->stopCharging());
        ASSERT_FALSE(tim_->enableEconomyMode());
        ASSERT_FALSE(tim_->disableEconomyMode());
        ASSERT_FALSE(tim_->acknowledgeAlarms());
    }

    crf::utility::logger::EventLogger logger_;
    std::string testDirName_;
    std::shared_ptr<crf::devices::siemensplc::SiemensPLCMock> plc_;
    bool isPLCConnected_;
    std::unique_ptr<crf::actuators::tim::TIMS300> tim_;

    const unsigned int datablockNumber4 = 4;
    const unsigned int datablockLength4 = 14;
    const unsigned int datablockNumber509 = 509;
    const unsigned int datablockLength509 = 7;
    const unsigned int datablockNumber510 = 510;
    const unsigned int datablockLength510 = 32;
    const unsigned int datablockNumber511 = 511;
    const unsigned int datablockLength511 = 37;

    const unsigned int  maxHeartbeatValue = 10000;

    const float obsStartPosition1_ = 10;
    const float obsEndPosition1_ = 50;
    const float obsMaximumVelocity1_ = 0.2;
    const bool obsMustRetractDevices1_ = true;
    const bool obsEnabled1_ = true;

    crf::actuators::tim::TIMAlarms mockAlarms_;
    crf::actuators::tim::TIMCommands mockCommands_;
    crf::actuators::tim::TIMSettings mockSettings_;
    crf::actuators::tim::TIMStatus mockStatus_;
};


TEST_F(TIMS300Should, initializeAndGetAllValues) {
    std::ifstream timData(testDirName_ + "timGoodConfig.json");
    nlohmann::json timJSON = nlohmann::json::parse(timData);
    tim_.reset(new crf::actuators::tim::TIMS300(timJSON, plc_));
    ASSERT_TRUE(tim_->initialize());
    getGoodTIMValues();
    ASSERT_TRUE(tim_->deinitialize());
}

TEST_F(TIMS300Should, initializeDeinitializeSequence) {
    std::ifstream timData(testDirName_ + "timGoodConfig.json");
    nlohmann::json timJSON = nlohmann::json::parse(timData);
    tim_.reset(new crf::actuators::tim::TIMS300(timJSON, plc_));

    ASSERT_TRUE(tim_->initialize());
    ASSERT_TRUE(tim_->initialize());
    ASSERT_TRUE(tim_->deinitialize());
    ASSERT_TRUE(tim_->deinitialize());
}

TEST_F(TIMS300Should, doesNotThrowExceptionWithoutPLCMockInitialized) {
    std::ifstream timData(testDirName_ + "timGoodConfig.json");
    nlohmann::json timJSON = nlohmann::json::parse(timData);
    tim_.reset(new crf::actuators::tim::TIMS300(timJSON));

    ASSERT_FALSE(tim_->initialize());

    getBadTIMValues();
    setBadTIMValues();

    ASSERT_TRUE(tim_->deinitialize());
}

TEST_F(TIMS300Should, allOperationFailsIfCouldNotInitialize) {
    ON_CALL(*plc_, initialize()).WillByDefault(testing::Return(false));
    ON_CALL(*plc_, deinitialize()).WillByDefault(testing::Return(false));

    std::ifstream timData(testDirName_ + "timGoodConfig.json");
    nlohmann::json timJSON = nlohmann::json::parse(timData);
    tim_.reset(new crf::actuators::tim::TIMS300(timJSON, plc_));

    ASSERT_FALSE(tim_->initialize());

    getBadTIMValues();
    setBadTIMValues();

    ASSERT_TRUE(tim_->deinitialize());
}

TEST_F(TIMS300Should, allOperationFailsIfConnectionIsLost) {
    std::ifstream timData(testDirName_ + "timGoodConfig.json");
    nlohmann::json timJSON = nlohmann::json::parse(timData);
    tim_.reset(new crf::actuators::tim::TIMS300(timJSON, plc_));

    ASSERT_TRUE(tim_->initialize());

    getGoodTIMValues();
    setGoodTIMValues();

    isPLCConnected_ = false;

    getBadTIMValues();
    setBadTIMValues();

    isPLCConnected_ = true;
}

TEST_F(TIMS300Should, moveFailsIfNotSafetToMove) {
    std::ifstream timData(testDirName_ + "timGoodConfig.json");
    nlohmann::json timJSON = nlohmann::json::parse(timData);
    tim_.reset(new crf::actuators::tim::TIMS300(timJSON, plc_));

    ASSERT_TRUE(tim_->initialize());
    mockStatus_.safeToMove(false);
    ASSERT_FALSE(tim_->moveToTarget(3200, 0.3).value());

    ASSERT_TRUE(tim_->deinitialize());
}
