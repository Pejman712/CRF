/* © Copyright CERN 2023.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <gtest/gtest.h>

#include "EventLogger/EventLogger.hpp"
#include "ViconAPI/ViconApiMockConfiguration.hpp"
#include "ViconAPI/ViconObject.hpp"
#include "ViconAPI/ViconMarker.hpp"
#include "MotionCapture/Vicon/Vicon.hpp"
#include "Types/Comparison.hpp"

using testing::_;

using crf::utility::types::areAlmostEqual;

class ViconMotionCaptureShould: public ::testing::Test {
 protected:
    ViconMotionCaptureShould():
      logger_("ViconMotionCaptureShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
        viconapi_.reset(
            new testing::NiceMock<crf::communication::viconapi::ViconApiMockConfiguration>);

        std::string testDirName = __FILE__;
        testDirName = testDirName.substr(0, testDirName.find("tests"));
        testDirName += "tests/config/";
        std::ifstream viconConfig(testDirName + "ViconConfig.json");
        viconJSON_ = nlohmann::json::parse(viconConfig);
    }

    ~ViconMotionCaptureShould() {
      logger_->info("{} END with {}",
          testing::UnitTest::GetInstance()->current_test_info()->name(),
          testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    void SetUp() override {
        viconapi_->configureMock();
        std::array<double, 3> desiredObjectTranslation = {50.232, 156.431, 245.187};
        // Rotation 45 degrees around x (w,x,y,z)
        std::array<double, 4> desiredObjectQuaternion = {0, 0.9238795, 0.3826834, 0};
        testObject_.objectName = "TestObject";
        testObject_.objectTranslation = desiredObjectTranslation;
        testObject_.objectQuaternion = desiredObjectQuaternion;
        testObject_.objectOccluded = false;

        std::array<double, 3> desiredMarkerPosition1 = {50.232, 120.431, 102.187};
        std::array<double, 3> desiredMarkerPosition2 = {40.232, 156.431, 0.187};
        std::array<double, 3> desiredMarkerPosition3 = {30.232, 75.431, 98.187};
        testObject_.objectMarkers.resize(3);
        testObject_.objectMarkers[0].markerName = "TestObject1";
        testObject_.objectMarkers[1].markerName = "TestObject2";
        testObject_.objectMarkers[2].markerName = "TestObject3";
        testObject_.objectMarkers[0].markerTranslation = desiredMarkerPosition1;
        testObject_.objectMarkers[1].markerTranslation = desiredMarkerPosition2;
        testObject_.objectMarkers[2].markerTranslation = desiredMarkerPosition3;
        testObject_.objectMarkers[0].markerOccluded = false;
        testObject_.objectMarkers[1].markerOccluded = false;
        testObject_.objectMarkers[2].markerOccluded = false;
    }

    crf::utility::logger::EventLogger logger_;
    std::string host_ = "194.12.147.119:801";
    nlohmann::json viconJSON_;
    crf::communication::viconapi::ViconObject testObject_;
    std::shared_ptr<crf::communication::viconapi::ViconApiMockConfiguration> viconapi_;
    std::unique_ptr<crf::sensors::motioncapture::ViconMotionCapture> sut_;
};


TEST_F(ViconMotionCaptureShould, returnTrueOnInitializeDeinitializeTest) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));

    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_TRUE(sut_->deinitialize());
}

TEST_F(ViconMotionCaptureShould, returnErrorIfNotConnected) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));

    viconapi_->goingToNotConnected();
    ASSERT_EQ((sut_->getObjectNames()).get_response(), crf::Code::Disconnected);
    ASSERT_EQ((sut_->getObjectPose("TestObject")).get_response(), crf::Code::Disconnected);
    ASSERT_EQ((sut_->getObjectMarkers("TestObject")).get_response(), crf::Code::Disconnected);
}

TEST_F(ViconMotionCaptureShould, returnErrorIfNotInitialized) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));

    ASSERT_EQ((sut_->getObjectNames()).get_response(), crf::Code::NotInitialized);
    ASSERT_EQ((sut_->getObjectPose("TestObject")).get_response(), crf::Code::NotInitialized);
    ASSERT_EQ((sut_->getObjectMarkers("TestObject")).get_response(), crf::Code::NotInitialized);
}

TEST_F(ViconMotionCaptureShould, returnErrorIfUnableToGetFrameInTime) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));
    ASSERT_TRUE(sut_->initialize());

    viconapi_->setGetFrameTimeoutExpired();
    ASSERT_EQ((sut_->getObjectNames()).get_response(), crf::Code::RequestTimeout);
    viconapi_->setGetFrameTimeoutExpired();
    ASSERT_EQ((sut_->getObjectPose("TestObject")).get_response(), crf::Code::RequestTimeout);
    viconapi_->setGetFrameTimeoutExpired();
    ASSERT_EQ((sut_->getObjectMarkers("TestObject")).get_response(), crf::Code::RequestTimeout);
}

TEST_F(ViconMotionCaptureShould, returnNegativeWhenGeneralViconErrorOccurs) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));
    ASSERT_TRUE(sut_->initialize());

    viconapi_->goingToError();
    ASSERT_FALSE(sut_->getObjectNames());
    ASSERT_FALSE(sut_->getObjectPose("TestObject"));
    ASSERT_FALSE(sut_->getObjectMarkers("TestObject"));
}

TEST_F(ViconMotionCaptureShould, returnValidObjectNamesList) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));
    ASSERT_TRUE(sut_->initialize());

    viconapi_->setViconObject(testObject_);
    std::vector<std::string> objectNameList = (sut_->getObjectNames()).value();
    for (unsigned int objectIndex = 0; objectIndex < objectNameList.size(); objectIndex++) {
        ASSERT_EQ(objectNameList[objectIndex], testObject_.objectName);
    }
}

TEST_F(ViconMotionCaptureShould, returnEmptyObjectNameListWhenNoObjects) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));
    ASSERT_TRUE(sut_->initialize());

    std::vector<std::string> objectList = (sut_->getObjectNames()).value();
    ASSERT_EQ(objectList.size(), 0);
}

TEST_F(ViconMotionCaptureShould, returnValidObjectPose) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));
    ASSERT_TRUE(sut_->initialize());

    viconapi_->setViconObject(testObject_);
    std::cout << "test" << std::endl;
    crf::utility::types::TaskPose desiredObjectPose(
        {testObject_.objectTranslation[0],
        testObject_.objectTranslation[1],
        testObject_.objectTranslation[2]},
        Eigen::Quaterniond(
        testObject_.objectQuaternion[0],
        testObject_.objectQuaternion[1],
        testObject_.objectQuaternion[2],
        testObject_.objectQuaternion[3]), 1e-7);
    crf::utility::types::TaskPose objectPose =
        (sut_->getObjectPose(testObject_.objectName)).value();
    ASSERT_TRUE(areAlmostEqual(objectPose, desiredObjectPose));
}

TEST_F(ViconMotionCaptureShould, returnValidMarkerList) {
    sut_.reset(new crf::sensors::motioncapture::ViconMotionCapture(host_, viconJSON_, viconapi_));
    ASSERT_TRUE(sut_->initialize());

    viconapi_->setViconObject(testObject_);
    std::vector<crf::sensors::motioncapture::MotionCaptureMarker> objectMarkerList =
        (sut_->getObjectMarkers(testObject_.objectName)).value();
    for (unsigned int markerIndex = 0; markerIndex < objectMarkerList.size(); markerIndex++) {
        ASSERT_EQ(objectMarkerList[markerIndex].markerName,
            testObject_.objectMarkers[markerIndex].markerName);
        ASSERT_EQ(objectMarkerList[markerIndex].markerTranslation,
            testObject_.objectMarkers[markerIndex].markerTranslation);
        ASSERT_EQ(objectMarkerList[markerIndex].markerOccluded,
            testObject_.objectMarkers[markerIndex].markerOccluded);
    }
}
