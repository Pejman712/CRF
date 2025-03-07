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

#include "ThermalCamera/FLIRCam.hpp"
#include "ThermalCamera/FLIRFactory.hpp"
#include "FLIRFactoryMock.hpp"

#define OPTRIS_CONFIG_FILE "../tests/ThermalCameraTests/17030203.xml"

using testing::InvokeWithoutArgs;
using testing::Return;
using testing::SaveArg;
using testing::_;
using testing::Invoke;


using crf::sensors::thermalcamera::IThermalCamera;
using crf::sensors::thermalcamera::FLIRFactory;
using crf::sensors::thermalcamera::FLIRCam;
using crf::sensors::thermalcamera::DeviceParams;
using crf::sensors::thermalcamera::TempConverter;
using crf::sensors::thermalcamera::FLIRFactoryMock;

class FLIRCamShould: public ::testing::Test {
 protected:
    FLIRCamShould() {
        ipcMock_.reset(new ::testing::NiceMock<IpcMock>);
        FLIRFactoryMock_.reset(new FLIRFactoryMock);
        ON_CALL(*FLIRFactoryMock_, selectThermalDevice(_)).WillByDefault(Invoke(
          [this](int index) {
            return realFactory_.selectThermalDevice(index);
          }));
        ON_CALL(*FLIRFactoryMock_, createDeviceParams(_)).WillByDefault(Invoke(
          [this](PvDevice *device) {
            return realFactory_.createDeviceParams(device);
          }));
        ON_CALL(*FLIRFactoryMock_, createTempConverter(_)).WillByDefault(Invoke(
          [this](DeviceParams* params) {
            return realFactory_.createTempConverter(params);
          }));
    }

    std::shared_ptr<::testing::NiceMock<IpcMock> > ipcMock_;
    std::shared_ptr<FLIRFactoryMock> FLIRFactoryMock_;
    FLIRFactory realFactory_;
    std::unique_ptr<IThermalCamera> sut_;
};


TEST_F(FLIRCamShould, returnEmptyFrameIfNotInitializedOnGetFrame) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_TRUE(sut_->getFrame().empty());
}
TEST_F(FLIRCamShould, returnFalseIfNotInitializedOnstream) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_FALSE(sut_->streamVideo());
}

TEST_F(FLIRCamShould, returnFalseIfAttemptToPublishEmptyFrame) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_FALSE(sut_->publishFrame(sut_->getFrame()));
}


// params

/*
 * TODO: think about all cases where FLIRFactoryMock_->xxx();
 * Should FlirFactory be tested?
 */

DeviceParams* customNullParamsAction(PvDevice* dev) {
    DeviceParams* params = nullptr;
    return params;
}
TEST_F(FLIRCamShould, returnFalseIfDeviceParamsNotCreated) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    EXPECT_CALL(*FLIRFactoryMock_, createDeviceParams(_)).WillOnce(
      Invoke(&customNullParamsAction));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(FLIRCamShould, returnFalseIfCreateConverterandNoParams) {
    DeviceParams *params = nullptr;
    TempConverter* converter;
    ASSERT_FALSE(FLIRFactoryMock_->createTempConverter(params));
}

TEST_F(FLIRCamShould, returnFalseIfCreateParamsandNoDevice) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    DeviceParams *params;
    PvDevice *device = nullptr;
    ASSERT_FALSE(FLIRFactoryMock_->createDeviceParams(device));
}

TempConverter* customNullTemConvAction(DeviceParams* params) {
    TempConverter* conv = nullptr;
    return conv;
}

/*
 * to be tested with camera DISCONNECTED
 */

TEST_F(FLIRCamShould, notFindAnyDeviceIfDeviceNotConnected) {
    ASSERT_EQ(0, FLIRFactoryMock_->selectThermalDevice(0)->GetLength());
}

/*
 * to be tested with a camera connected
 */

TEST_F(FLIRCamShould, DISABLED_returnFalseIfAlreadyInitialized) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(FLIRCamShould, DISABLED_returnFalseIfAlreadyDeinitialized) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(FLIRCamShould, DISABLED_returnTrueIfInitialized) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_TRUE(sut_->initialize());
}

TEST_F(FLIRCamShould, DISABLED_returnTrueifCameraConnected) {
    PvString *aConnectionID;
    ASSERT_TRUE(FLIRFactoryMock_->selectThermalDevice(0)->GetLength() > 0);  // NOLINT
}

TEST_F(FLIRCamShould, DISABLED_returnTrueWhenPublishCorrectFrame) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_TRUE(sut_->initialize());
    // according to FLIR camera resolution
    unsigned int expectedHeight = 240;
    unsigned int expectedWidth = 320;
    cv::Mat resultFrame = sut_->getFrame();
    ASSERT_EQ(expectedHeight, resultFrame.rows);
    ASSERT_EQ(expectedWidth, resultFrame.cols);
    cv::Point * p = nullptr;
    // check if temperature values make sense
    cv::checkRange(resultFrame, true, p, -30, 50);
    ASSERT_TRUE(!p);
    ASSERT_TRUE(sut_->publishFrame(resultFrame));
}

TEST_F(FLIRCamShould, DISABLED_returnValidData) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_TRUE(sut_->initialize());
    // according to FLIR camera resolution
    unsigned int expectedHeight = 240;
    unsigned int expectedWidth = 320;
    cv::Mat resultFrame = sut_->getFrame();
    ASSERT_EQ(expectedHeight, resultFrame.rows);
    ASSERT_EQ(expectedWidth, resultFrame.cols);
    cv::Point * p;
    // check if temperature values make sense
    cv::checkRange(resultFrame, true, p, -30, 50);
    ASSERT_TRUE(!p);
}

// params

TEST_F(FLIRCamShould, DISABLED_returnFalseOnInvalidDoubleParam) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));

    DeviceParams* params = nullptr;
    EXPECT_CALL(*FLIRFactoryMock_, createDeviceParams(_)).WillOnce(Invoke([this, &params](PvDevice *device) {  // NOLINT
        params = realFactory_.createDeviceParams(device);  // lambda capture block
        return params;
    }));
    ASSERT_TRUE(sut_->initialize());
    double val;
    ASSERT_FALSE(params == nullptr);
    ASSERT_FALSE(params->get_double_param("xyz", &val));
}

TEST_F(FLIRCamShould, DISABLED_returnTrueOnValidDoubleParam) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));

    DeviceParams* params = nullptr;
    EXPECT_CALL(*FLIRFactoryMock_, createDeviceParams(_)).WillOnce(Invoke([this, &params](PvDevice *device) {  // NOLINT
        params = realFactory_.createDeviceParams(device);  // lambda capture block
        return params;
    }));
    ASSERT_TRUE(sut_->initialize());
    double val;
    ASSERT_TRUE(params->get_double_param("B", &val));
}

TEST_F(FLIRCamShould, DISABLED_returnFalseOnWrongParamType) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    DeviceParams* params = nullptr;
    EXPECT_CALL(*FLIRFactoryMock_, createDeviceParams(_)).WillOnce(Invoke([this, &params](PvDevice *device) {  // NOLINT
        params = realFactory_.createDeviceParams(device);  // lambda capture block
        return params;
    }));
    ASSERT_TRUE(sut_->initialize());
    double val;
    ASSERT_FALSE(params->get_double_param("J0", &val));
}

TEST_F(FLIRCamShould, DISABLED_streamVideoWithRealDevice) {
    sut_.reset(new FLIRCam(ipcMock_, FLIRFactoryMock_));
    ASSERT_TRUE(sut_->initialize());

    // std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->initialize());

    // std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(100));
    ASSERT_TRUE(sut_->streamVideo());
}
