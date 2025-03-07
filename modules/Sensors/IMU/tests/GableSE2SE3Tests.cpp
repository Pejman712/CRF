/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
*/

#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "EtherCATDrivers/EtherCATMaster/EtherCATMaster.hpp"
#include "IMU/Gable/SE2SE3/GableSE2SE3DriverMockConfiguration.hpp"
#include "IMU/Gable/SE2SE3/GableSE2SE3.hpp"
#include "IMU/Gable/GableInfo.hpp"

using testing::_;
using testing::NiceMock;

class GableSE2SE3Should: public ::testing::Test {
 protected:
    GableSE2SE3Should():
        imuDriver_(std::make_shared<NiceMock<
            crf::sensors::imu::GableSE2SE3DriverMockConfiguration>>(
            numberOfSlaves_, sizeOfIOMap_)),
        logger_("GableSE2SE3Should") {
        logger_->info(
            "{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~GableSE2SE3Should() {
        logger_->info(
            "{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        imuDriver_->configure();
        master_ = std::make_shared<crf::devices::ethercatdrivers::EtherCATMaster>(
            "ifname", numberOfSlaves_, cycleTime_, sizeOfIOMap_, imuDriver_);

        sut_ = std::make_unique<crf::sensors::imu::GableSE2SE3>(master_, 1);
    }

    void TearDown() override {
        logger_->info("TearDown");
    }

    const int numberOfSlaves_ = 1;
    const std::chrono::microseconds cycleTime_ = std::chrono::milliseconds(1);
    const int sizeOfIOMap_ = 137;

    std::shared_ptr<crf::devices::ethercatdrivers::EtherCATMaster> master_;
    std::shared_ptr<crf::sensors::imu::GableSE2SE3DriverMockConfiguration> imuDriver_;
    std::unique_ptr<crf::sensors::imu::GableSE2SE3> sut_;
    crf::utility::logger::EventLogger logger_;
};

TEST_F(GableSE2SE3Should, successfullyReturnSignal) {
    ASSERT_TRUE(master_->initialize());
    ASSERT_TRUE(sut_->initialize());
    crf::sensors::imu::IMUSignals signal = sut_->getSignal();
    std::array<double, 4> expectedOrientation = {0.0, 0.9238795, 0.3826834, 0.0};  // w,x,y,z
    std::array<double, 3> expectedEulerZYX = {45.0, 0.0, 180.0};  // eulerZYX in deg
    std::array<double, 3> expectedAngularVelocity = {1.8, 0.12, 2.5};
    std::array<double, 3> expectedLinearAcceleration = {0.23, 1.15, 0.68};
    std::array<double, 3> expectedMagneticField = {0.53, 0.27, 1.3};
    for (int i=0; i < 4; i++) {
        ASSERT_NEAR(signal.quaternion.value()[0], expectedOrientation[0], 1e-3);
        ASSERT_NEAR(signal.quaternion.value()[1], expectedOrientation[1], 1e-3);
        ASSERT_NEAR(signal.quaternion.value()[2], expectedOrientation[2], 1e-3);
        ASSERT_NEAR(signal.quaternion.value()[3], expectedOrientation[3], 1e-3);
    }
    for (int i=0; i < 3; i++) {
        ASSERT_NEAR(signal.eulerZYX.value()[0], expectedEulerZYX[0], 1e-3);
        ASSERT_NEAR(signal.eulerZYX.value()[1], expectedEulerZYX[1], 1e-3);
        ASSERT_NEAR(signal.eulerZYX.value()[2], expectedEulerZYX[2], 1e-3);
        ASSERT_NEAR(signal.angularVelocity.value()[0], expectedAngularVelocity[0], 1e-3);
        ASSERT_NEAR(signal.angularVelocity.value()[1], expectedAngularVelocity[1], 1e-3);
        ASSERT_NEAR(signal.angularVelocity.value()[2], expectedAngularVelocity[2], 1e-3);
        ASSERT_NEAR(signal.linearAcceleration.value()[0], expectedLinearAcceleration[0], 1e-3);
        ASSERT_NEAR(signal.linearAcceleration.value()[1], expectedLinearAcceleration[1], 1e-3);
        ASSERT_NEAR(signal.linearAcceleration.value()[2], expectedLinearAcceleration[2], 1e-3);
        ASSERT_NEAR(signal.magneticField.value()[0], expectedMagneticField[0], 1e-3);
        ASSERT_NEAR(signal.magneticField.value()[1], expectedMagneticField[1], 1e-3);
        ASSERT_NEAR(signal.magneticField.value()[2], expectedMagneticField[2], 1e-3);
    }
    ASSERT_EQ(signal.position.get_response(), crf::Code::NotImplemented);
    ASSERT_EQ(signal.linearVelocity.get_response(), crf::Code::NotImplemented);
    ASSERT_EQ(signal.angularAcceleration.get_response(), crf::Code::NotImplemented);
}

TEST_F(GableSE2SE3Should, returnErrorCodeWhenCalibrating) {
    ASSERT_TRUE(master_->initialize());
    ASSERT_TRUE(sut_->initialize());
    crf::expected<bool> result = sut_->calibrate();
    ASSERT_EQ(result.get_response(), crf::Code::MethodNotAllowed);
}

TEST_F(GableSE2SE3Should, successfullyReturnInfos) {
    ASSERT_TRUE(master_->initialize());
    ASSERT_TRUE(sut_->initialize());
    imuDriver_->setMessageID(100);
    crf::expected<crf::sensors::imu::GableInfo> info = sut_->getInfo();
    ASSERT_EQ(info.value().Baudrate, 12);
    ASSERT_EQ(info.value().Firmware, 1);
    ASSERT_EQ(info.value().Hardware, 3);
    ASSERT_EQ(info.value().FilterProfile.value(), 50);
    ASSERT_EQ(info.value().DeviceID, 59280700);
}

TEST_F(GableSE2SE3Should, successfullyEnableAHS) {
    ASSERT_TRUE(master_->initialize());
    ASSERT_TRUE(sut_->initialize());
    imuDriver_->setMessageID(72);
    bool resultEnable = sut_->enableAHS();
    ASSERT_EQ(resultEnable, true);
}

TEST_F(GableSE2SE3Should, successfullySetFilter) {
    ASSERT_TRUE(master_->initialize());
    ASSERT_TRUE(sut_->initialize());
    imuDriver_->setMessageID(100);
    bool result = sut_->setFilter(crf::sensors::imu::FilterProfile::Dynamic);
    ASSERT_EQ(result, true);
}
