/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <memory>
#include <thread>

#include "EventLogger/EventLogger.hpp"

#include "Radars/XethruRadar/XethruAdapter.hpp"

#define TIME_IN_SEC_FOR_DEVICE_RESTART 5

using crf::sensors::xethruradar::IXethruAdapter;
using crf::sensors::xethruradar::XethruAdapter;


class XethruAdapterShould: public ::testing::Test{
 protected:
    XethruAdapterShould():
    logger_("XethruAdapterShould") {
    }
    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<IXethruAdapter> sut_;
};

TEST_F(XethruAdapterShould, DISABLED_returnFullFrameIfInitialized) {
    std::this_thread::sleep_for(std::chrono::seconds(TIME_IN_SEC_FOR_DEVICE_RESTART));
    sut_.reset(new XethruAdapter("/dev/ttyACM0"));
    ASSERT_TRUE(sut_->initialize());

    float fa1 = 0.4;
    float fa2 = 5.0;
    int sensitivity = 9;

    ASSERT_TRUE(sut_->startStream(fa1, fa2, sensitivity));
    XeThru::RespirationData testData = sut_->getFrame();
    EXPECT_NE(0, testData.frame_counter);
}

TEST_F(XethruAdapterShould, DISABLED_returnFalseIfInitializedTwice) {
    std::this_thread::sleep_for(std::chrono::seconds(TIME_IN_SEC_FOR_DEVICE_RESTART));
    sut_.reset(new XethruAdapter("/dev/ttyACM0"));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(XethruAdapterShould, DISABLED_returnEmptyFrameIfNotStreaming) {
    std::this_thread::sleep_for(std::chrono::seconds(TIME_IN_SEC_FOR_DEVICE_RESTART));
    sut_.reset(new XethruAdapter("/dev/ttyACM0"));
    ASSERT_TRUE(sut_->initialize());
    XeThru::RespirationData testData = sut_->getFrame();
    EXPECT_EQ(0, testData.frame_counter);
}
