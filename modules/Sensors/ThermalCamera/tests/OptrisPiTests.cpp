/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include <libirimager/IRImagerClient.h>
#include <opencv2/core.hpp>

#include "Mocks/Communication/IpcMock.hpp"
#include "Mocks/Sensors/IRDeviceMock.hpp"
#include "Mocks/Sensors/OptrisDeviceFactoryMock.hpp"

#include "ThermalCamera/OptrisDeviceFactory.hpp"  // for HW tests
#include "ThermalCamera/OptrisPi.hpp"

#define OPTRIS_CONFIG_FILE "../tests/ThermalCameraTests/17030203.xml"

using testing::InvokeWithoutArgs;
using testing::Return;
using testing::SaveArg;
using testing::_;

using crf::sensors::thermalcamera::IOptrisDeviceFactory;
using crf::sensors::thermalcamera::IThermalCamera;
using crf::sensors::thermalcamera::OptrisDeviceFactoryMock;
using crf::sensors::thermalcamera::OptrisDeviceFactory;
using crf::sensors::thermalcamera::OptrisPi;

class OptrisPiShould: public ::testing::Test {
 protected:
    OptrisPiShould() {
        ipcMock_.reset(new IpcMock);
        ON_CALL(*ipcMock_, write(_, _)).WillByDefault(Return(true));
        optrisDeviceFactoryMock_.reset(new OptrisDeviceFactoryMock);
        irDeviceMock_.reset(new evo::IRDeviceMock);
        ON_CALL(*optrisDeviceFactoryMock_, createDevice()).WillByDefault(Return(irDeviceMock_));
        /*
         * The IRImager class crashes if "init" method is not called after construction.
         * It also crashes if init is called, but params are not consistent.
         * Consequently, I'd rather just return nullptr and not test the part of code
         * that relies on IRImager. It is used just to delegate call from rawFrame callback
         * to thermalFrame callback, anyway ...
         */
        ON_CALL(*optrisDeviceFactoryMock_, createImager(_, _, _)).WillByDefault(Return(nullptr));
    }

    std::shared_ptr<IpcMock> ipcMock_;
    std::shared_ptr<OptrisDeviceFactoryMock> optrisDeviceFactoryMock_;
    std::shared_ptr<evo::IRDeviceMock> irDeviceMock_;
    std::unique_ptr<IThermalCamera> sut_;
};

TEST_F(OptrisPiShould, returnFalseIfInitializedOrDeinitializedTwice) {
    sut_.reset(new OptrisPi(ipcMock_, optrisDeviceFactoryMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(OptrisPiShould, notCallDeviceIfNotInitialized) {
    evo::IRImagerClient* callback;
    EXPECT_CALL(*irDeviceMock_, setClient(_)).Times(0);
    sut_.reset(new OptrisPi(ipcMock_, optrisDeviceFactoryMock_));
}

TEST_F(OptrisPiShould, returnEmptyFrameIfNoDevice) {
    EXPECT_CALL(*optrisDeviceFactoryMock_, createDevice()).WillOnce(Return(nullptr));
    sut_.reset(new OptrisPi(ipcMock_, optrisDeviceFactoryMock_));
    sut_->initialize();
    ASSERT_TRUE(sut_->getFrame().empty());
}

TEST_F(OptrisPiShould, notCrashWhenCallbacksInvoked) {
    evo::IRImagerClient* callback;
    EXPECT_CALL(*irDeviceMock_, setClient(_)).WillOnce(SaveArg<0>(&callback));
    sut_.reset(new OptrisPi(ipcMock_, optrisDeviceFactoryMock_));
    sut_->initialize();
    callback->onRawFrame(nullptr, 0);
    callback->onThermalFrame(nullptr, 0, 0, {}, nullptr);
    callback->onThermalFrameEvent(nullptr, 0, 0, {}, nullptr);
    callback->onVisibleFrame(nullptr, 0, 0, {}, nullptr);
    callback->onVisibleFrameEvent(nullptr, 0, 0, {}, nullptr);
    callback->onFlagStateChange({}, nullptr);
}

TEST_F(OptrisPiShould, returnCorrectFrameIfThermalCallbackInvoked) {
    float accuracy = 1e-6;
    evo::IRImagerClient* callback;
    EXPECT_CALL(*irDeviceMock_, setClient(_)).WillOnce(SaveArg<0>(&callback));
    sut_.reset(new OptrisPi(ipcMock_, optrisDeviceFactoryMock_));
    sut_->initialize();

    unsigned int expectedHeight = 2;
    unsigned int expectedWidth = 8;
    unsigned short* rawData = new unsigned short[expectedWidth*expectedHeight];  // NOLINT
    for (int i = 0; i < expectedHeight*expectedWidth; i++) {
        rawData[i] = i;
    }
    float* expectedThermalData = new float[expectedWidth*expectedHeight];
    std::transform(rawData, rawData + expectedWidth*expectedHeight, expectedThermalData,
        [](unsigned short elem) {  // NOLINT
            return ((((float)elem)-1000.f))/10.f;  // formula from the docs NOLINT
        });

    // first one with some bad data
    callback->onThermalFrame(rawData, 0, 0, {}, nullptr);
    // second one with some nice data
    callback->onThermalFrame(rawData, expectedWidth, expectedHeight, {}, nullptr);

    cv::Mat resultFrame = sut_->getFrame();
    ASSERT_EQ(expectedHeight, resultFrame.rows);
    ASSERT_EQ(expectedWidth, resultFrame.cols);

    for (int i = 0; i < expectedHeight; i++) {
        for (int j = 0; j < expectedWidth; j++) {
            EXPECT_NEAR(expectedThermalData[i*expectedWidth + j],
                resultFrame.at<float>(i, j), accuracy) << "where i: " << i
                << ", j: " << j;
        }
    }

    delete[] rawData;
    delete[] expectedThermalData;
}

TEST_F(OptrisPiShould, returnFalseIfAttemptToPublishEmptyFrame) {
    evo::IRImagerClient* callback;
    EXPECT_CALL(*irDeviceMock_, setClient(_)).WillOnce(SaveArg<0>(&callback));
    sut_.reset(new OptrisPi(ipcMock_, optrisDeviceFactoryMock_));
    sut_->initialize();
    callback->onThermalFrame(nullptr, 0, 0, {}, nullptr);
    ASSERT_FALSE(sut_->publishFrame(sut_->getFrame()));
}

TEST_F(OptrisPiShould, returnTrueWhenPublishCorrectFrame) {
    evo::IRImagerClient* callback;
    EXPECT_CALL(*irDeviceMock_, setClient(_)).WillOnce(SaveArg<0>(&callback));
    sut_.reset(new OptrisPi(ipcMock_, optrisDeviceFactoryMock_));
    sut_->initialize();
    unsigned int expectedHeight = 2;
    unsigned int expectedWidth = 8;
    unsigned short* rawData = new unsigned short[expectedWidth*expectedHeight];  // NOLINT
    for (int i = 0; i < expectedHeight*expectedWidth; i++) {
        rawData[i] = i;
    }

    callback->onThermalFrame(rawData, expectedWidth, expectedHeight, {}, nullptr);
    ASSERT_TRUE(sut_->publishFrame(sut_->getFrame()));

    delete[] rawData;
}

/*
 * You can run the following 2 testcases if you have an Optris device connected
 * to your PC.
 */
TEST_F(OptrisPiShould, DISABLED_showFrameWithRealDevice) {
    std::string configFile(OPTRIS_CONFIG_FILE);
    auto factory = std::make_shared<OptrisDeviceFactory>(configFile);
    sut_.reset();  // free the resources first
    sut_.reset(new OptrisPi(ipcMock_, factory));
    sut_->initialize();
    ASSERT_TRUE(true);
    // wait for camera to capturea good picture
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
    ASSERT_TRUE(sut_->showFrame());
    // let's capture a second frame, because sometimes first one is not nice
    ASSERT_TRUE(sut_->showFrame());
}

TEST_F(OptrisPiShould, DISABLED_streamVideoWithRealDevice) {
    std::string configFile(OPTRIS_CONFIG_FILE);
    auto factory = std::make_shared<OptrisDeviceFactory>(configFile);
    sut_.reset();  // free the resources first
    sut_.reset(new OptrisPi(ipcMock_, factory));
    sut_->initialize();
    /*
     * WARNING!!!
     * Apparently OptrisPi doesn't like to be switched on/off repeatedly without
     * being given some time to warm up. You can initialize/deinitialize device
     * multiple times only if you maintain some reasonable interval between these
     * operations.
     *
     * Last time I tried to initialize/deinitialize the device without any sleep between
     * I tainted the kernel module responsible for video
     */
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
    sut_->deinitialize();
    sut_->initialize();
    // wait for camera to start streaming
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
    ASSERT_TRUE(sut_->streamVideo());
}
