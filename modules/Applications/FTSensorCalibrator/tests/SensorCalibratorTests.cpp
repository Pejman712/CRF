/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <vector>
#include <nlohmann/json.hpp>

#include "FTSensorCalibrator/FTSensorCalibrator.hpp"
#include "Mocks/Applications/RobotArmControllerDeprecatedMock.hpp"
#include "Mocks/Sensors/FTSensorMock.hpp"

using crf::applications::ftsensorcalibrator::FTSensorCalibrator;
using crf::sensors::ftsensor::FTSensorMock;
using crf::applications::robotarmcontroller::RobotArmControllerDeprecatedMock;
using crf::utility::types::JointPositions;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskForceTorque;

using json = nlohmann::json;

using testing::_;
using testing::NiceMock;
using testing::Invoke;
using testing::Return;

class FTSensorCalibratorShould: public ::testing::Test {
 protected:
    FTSensorCalibratorShould(): logger_("FTSensorCalibratorShould"),
                                biases_(6),
                                weight_(2),
                                currentJP_(6),
                                currentCP_() {
        logger_->info("{} BEGIN",
                      testing::UnitTest::GetInstance()->current_test_info()->name());
        calibrationConfig_ = __FILE__;
        calibrationConfig_ = calibrationConfig_.substr(0,
             calibrationConfig_.find("SensorCalibratorTests.cpp"));
        calibrationConfig_ += "config/SchunkArmCalibration.json";
        sensorMock_.reset(new NiceMock<FTSensorMock>);
        armMock_.reset(new NiceMock<RobotArmControllerDeprecatedMock>);

        ON_CALL(*armMock_, setJointPositions(_)).WillByDefault(
                Invoke([this](const JointPositions& jp){return saveJointsPos(jp);}));
        ON_CALL(*armMock_, setTaskPose(_)).WillByDefault(
            Invoke([this](const TaskPose& cp){return saveTaskPos(cp);}));
        ON_CALL(*armMock_, getJointPositions()).WillByDefault(
                Invoke([this](){return returnJointsPos();}));
        ON_CALL(*armMock_, getTaskPose()).WillByDefault(
                Invoke([this](){return returnTaskPos();}));

        ON_CALL(*sensorMock_, updateBias(_, _)).WillByDefault(Return(true));
        ON_CALL(*sensorMock_, isCalibrated()).WillByDefault(Return(true));
        ON_CALL(*sensorMock_, getRawFT()).WillByDefault(
            Invoke([this](){return returnFTMeasurement();}));
    }
    ~FTSensorCalibratorShould() {
        logger_->info("{} END with {}",
                      testing::UnitTest::GetInstance()->current_test_info()->name(),
                      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    bool saveJointsPos(const JointPositions& jp) {
        currentJP_ = jp;
        return true;
    }
    JointPositions returnJointsPos() {
        return currentJP_;
    }
    bool saveTaskPos(const TaskPose& cp) {
        currentCP_ = cp;
        return true;
    }
    TaskPose returnTaskPos() {
        return currentCP_;
    }
    TaskForceTorque returnFTMeasurement() {
        TaskForceTorque ct;
        for (int i = 0; i < 6; i++) {
            ct(i) = biases_[i];
        }
        // simulate mass
        ct(0) += sin(currentJP_(4)) * weight_[0];
        ct(1) += cos(currentJP_(4)) * weight_[0];
        ct(3) += sin(currentJP_(4)) * weight_[1];
        return ct;
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<FTSensorCalibrator> sut_;
    std::shared_ptr<FTSensorMock> sensorMock_;
    std::shared_ptr<RobotArmControllerDeprecatedMock> armMock_;
    std::string calibrationConfig_;

    // simulated parameters
    std::vector<float> biases_;
    std::vector<float> weight_;
    JointPositions currentJP_;
    TaskPose currentCP_;
};

TEST_F(FTSensorCalibratorShould, initDeinitWorksFine) {
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(FTSensorCalibratorShould, deinitSetsPositionToTheStart) {
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));

    EXPECT_CALL(*armMock_, setJointPositions(JointPositions(6))).WillRepeatedly(Return(true));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    JointPositions jp2({0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    EXPECT_CALL(*armMock_, getJointPositions()).WillRepeatedly(Return(jp2));
    ASSERT_TRUE(sut_->initialize());
    EXPECT_CALL(*armMock_, setJointPositions(jp2)).WillOnce(Return(true));
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(FTSensorCalibratorShould, initFailsWhenWrongConfig) {
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());

    // Wrong file path
    calibrationConfig_ = calibrationConfig_.substr(0, calibrationConfig_.size()-10);
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(FTSensorCalibratorShould, calibrateWithoutLoad) {
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));
    ASSERT_FALSE(sut_->calibrate());

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->calibrate());
    std::ifstream inCalibFile;
    inCalibFile.open(sut_->getCalibrationConfig().getLogFile());

    json json;
    ASSERT_NO_THROW(json = json::parse(inCalibFile));
    for (int i =0; i < 6; i++) {
        ASSERT_FLOAT_EQ(0, json["biases"][i]);
    }
    ASSERT_FLOAT_EQ(0, json["GravityCompensation"]["weight"]);
    ASSERT_FLOAT_EQ(0, json["GravityCompensation"]["torque"]);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(FTSensorCalibratorShould, calibrateWithLoad) {
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));
    // setting biasses and weights
    for (int i = 0; i < 6; i++) {
        biases_[i] = 25;
    }
    weight_[0] = 7;
    weight_[1] = 9;

    ASSERT_TRUE(sut_->initialize());

    ASSERT_TRUE(sut_->calibrate());
    std::ifstream inCalibFile;
    inCalibFile.open(sut_->getCalibrationConfig().getLogFile());

    json json;
    ASSERT_NO_THROW(json = json::parse(inCalibFile));
    for (int i =0; i < 6; i++) {
        ASSERT_FLOAT_EQ(biases_[i], json["biases"][i]);
    }
    ASSERT_FLOAT_EQ(weight_[0], json["GravityCompensation"]["weight"]);
    ASSERT_FLOAT_EQ(weight_[1], json["GravityCompensation"]["torque"]);
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(FTSensorCalibratorShould, checkValidationHappyPath) {
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->calibrate());

    TaskForceTorque ct{};
    EXPECT_CALL(*sensorMock_, getFTGravityFree(_, _)).
                        Times(2).WillRepeatedly(Return(ct));
    ASSERT_TRUE(sut_->validateCalibration());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(FTSensorCalibratorShould, validationReturnFalseWhenUpdateBiasFails) {
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));
    ASSERT_FALSE(sut_->validateCalibration());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->calibrate());

    TaskForceTorque ct{};
    EXPECT_CALL(*sensorMock_, isCalibrated()).WillRepeatedly(Return(false));
    ASSERT_FALSE(sut_->validateCalibration());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(FTSensorCalibratorShould, validationReturnFalseWhenForcesTooBig) {
    sut_.reset(new FTSensorCalibrator(armMock_, sensorMock_, calibrationConfig_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->calibrate());

    TaskForceTorque ct{10, 10, 10, 10, 10, 10};
    EXPECT_CALL(*sensorMock_, getFTGravityFree(_, _)).WillOnce(Return(ct));
    ASSERT_FALSE(sut_->validateCalibration());

    EXPECT_CALL(*sensorMock_, getFTGravityFree(_, _)).WillOnce(Return(TaskForceTorque{})).
                    WillOnce(Return(ct));
    ASSERT_FALSE(sut_->validateCalibration());

    EXPECT_CALL(*sensorMock_, getFTGravityFree(_, _)).Times(2).
                    WillRepeatedly(Return(TaskForceTorque{}));
    ASSERT_TRUE(sut_->validateCalibration());
    ASSERT_TRUE(sut_->deinitialize());
}
