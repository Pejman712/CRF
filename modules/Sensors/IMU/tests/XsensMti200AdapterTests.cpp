/* Copyright 2017 CERN */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <sys/sysmacros.h>

#include "EventLogger/EventLogger.hpp"
#include "XsensMT/XsensMT.hpp"

using crf::sensors::imu::XsensMT;
using ::testing::_;
using ::testing::Return;

MATCHER_P(headersEqual, expectedHeader, "") {
    bool equal = expectedHeader.type == arg.type && expectedHeader.length == arg.length;
    if (!equal) {
        *result_listener << "arg.type: " << arg.type << ", arg.length: " << arg.length;
    }
    return equal;
}
/*
 * Unfortunately, XSENS does not provide any abstract interfaces for their objects,
 * and consequently this class is almost not testable on UT level
 * Any ideas on how to mock XsControl are welcome!
 */
class XsensMti200AdapterShould: public ::testing::Test {
 protected:
    XsensMti200AdapterShould() {
        timeout_ = 1000;
        sleepInterval_ = 100;
        sut_.reset(new XsensMT(XsControl::construct()));
    }
    ~XsensMti200AdapterShould() {
        sut_.reset();
    }
    void waitForFreshData();
    std::unique_ptr<XsensMT> sut_;
    int timeout_;
    int sleepInterval_;
};

void XsensMti200AdapterShould::waitForFreshData() {
    auto t_start = std::chrono::high_resolution_clock::now();
    while (!sut_->hasFreshData()) {
        auto dur = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t_start).count();
        ASSERT_LE(dur, timeout_);
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleepInterval_));
    }
}

TEST_F(XsensMti200AdapterShould, DISABLED_returnFalseOnCallToHasFreshDataifDeviceUninitialized) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->hasFreshData());
}

TEST_F(XsensMti200AdapterShould, DISABLED_returnSomeFreshDataNoLaterThanSecondAfterInitializing) {
    ASSERT_TRUE(sut_->initialize());
    waitForFreshData();
    // Since we are operating on a live object here, we check for non-zero acc
    Packets::IMUDataPacket receivedPacket = sut_->getIMUDataPacket();
    bool nonZeroAcceleration = (receivedPacket.acceleration[0] != 0
        || receivedPacket.acceleration[1] != 0
        || receivedPacket.acceleration[2] != 0);
    ASSERT_TRUE(nonZeroAcceleration);
}

TEST_F(XsensMti200AdapterShould, DISABLED_returnFalseIfDataIsAlreadyReadAndHardwareClosed) {
    ASSERT_TRUE(sut_->initialize());
    waitForFreshData();
    ASSERT_TRUE(sut_->deinitialize());
    sut_->getIMUDataPacket();
    ASSERT_FALSE(sut_->hasFreshData());
}

/* DISABLED by default; should be run only for performance testing */
TEST_F(XsensMti200AdapterShould, DISABLED_doPerformanceTest) {
    /* test time = numSamples*sampleInterval */
    int numSamples = 180000;  // 180000*0.02 = 3600s = 1h
    int sampleInterval = 20;  // msec
    crf::utility::logger::EventLogger logger_("XsensMti200AdapterShould");

    ASSERT_TRUE(sut_->initialize());
    for (int i=0; i < numSamples; i++) {
        if (sut_->hasFreshData()) {
            logger_->info("Data[{}]: {}", i, sut_->getIMUDataPacket());
        }
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sampleInterval));
    }
}
