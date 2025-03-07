/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "boost/optional.hpp"

#include "CommUtility/StreamWriter.hpp"
#include "PersonFollower/PersonDetector.hpp"
#include "VisionTypes/VisionTypes.hpp"

#include "Mocks/Communication/RequestMock.hpp"
#include "Mocks/Sensors/CameraMock.hpp"
#include "Mocks/Vision/ObjectDetectorMock.hpp"

using crf::applications::personfollower::PersonDetector;
using crf::sensors::cameras::CameraMock;
using crf::vision::types::BoundingBox;
using crf::vision::objectDetection::ObjectDetectorMock;

using ::testing::Invoke;
using testing::_;
using testing::NiceMock;
using testing::SetArgReferee;


class PersonDetectionShould: public ::testing::Test {
 protected:
    PersonDetectionShould(): logger_("PersonDetectionShould"),
        testAddress_(__FILE__),
        emptyMat_(cv::Mat::zeros(cv::Size(500, 500), CV_32FC3)),
        expectedBbox_({234, 90, 270, 104}),
        ExpectedImageWidth_(250) {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
        cameraMock_.reset(new NiceMock<CameraMock>);
        detector_.reset(new NiceMock<ObjectDetectorMock>());
        ON_CALL(*detector_, initialize()).WillByDefault(::testing::Return(true));
        ON_CALL(*detector_, deinitialize()).WillByDefault(::testing::Return(true));
        testAddress_ = testAddress_.substr(0, testAddress_.find("PersonDetectorTests.cpp"));
        ON_CALL(*cameraMock_, initialize()).WillByDefault(::testing::Return(true));
        ON_CALL(*cameraMock_, deinitialize()).WillByDefault(::testing::Return(true));
        ON_CALL(*cameraMock_, getFrame()).WillByDefault(::testing::Return(emptyMat_));
    }
    void SetUp() override {
        sut_.reset(new PersonDetector(cameraMock_, detector_));
        BoundingBox bbox;
        bbox.box = {264, 143, 413, 360};
        bbox.classif = "person";
        personBbox_.push_back(bbox);
    }

    ~PersonDetectionShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::string testAddress_;
    cv::Mat emptyMat_;
    cv::Rect2d expectedBbox_;
    int ExpectedImageWidth_;
    std::shared_ptr<CameraMock> cameraMock_;
    std::shared_ptr<ObjectDetectorMock> detector_;
    std::unique_ptr<PersonDetector> sut_;
    std::vector<BoundingBox> personBbox_;
};

TEST_F(PersonDetectionShould, ReturnFalseIfAlreadyRunningAndShouldNotStartTwice) {
    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());

    ASSERT_TRUE(sut_->initialize());
    ASSERT_FALSE(sut_->initialize());
    ASSERT_TRUE(sut_->deinitialize());
    ASSERT_FALSE(sut_->deinitialize());
}

TEST_F(PersonDetectionShould, ReturnImageCenterWidth) {
    EXPECT_CALL(*cameraMock_, getFrame()).WillOnce(::testing::Return(emptyMat_));
    ASSERT_TRUE(sut_->initialize());
    int receivedImageCenterWidth = sut_->getImageCenterWidth();
    ASSERT_EQ(ExpectedImageWidth_, receivedImageCenterWidth);
}

TEST_F(PersonDetectionShould, returnEmptyRoiIfNoBoxesFound) {
    EXPECT_CALL(*cameraMock_, getFrame()).WillOnce(::testing::Return(emptyMat_));
    EXPECT_CALL(*detector_, getBoundingBoxes(_)).WillOnce(Invoke(
        [](cv::Mat) {
            std::vector<BoundingBox> bboxes;
            return bboxes;
        }));
    ASSERT_TRUE(sut_->initialize());
    cv::Rect2d roi = sut_->getPersonBoundingBox();
    ASSERT_TRUE(roi.empty());
}

TEST_F(PersonDetectionShould, returnRoiIfPersonFound) {
    EXPECT_CALL(*cameraMock_, getFrame()).WillOnce(::testing::Return(emptyMat_));
    EXPECT_CALL(*detector_, getBoundingBoxes(_)).WillOnce(Invoke(
        [this](cv::Mat) {
            return personBbox_;
        }));
    ASSERT_TRUE(sut_->initialize());
    cv::Rect2d roi = sut_->getPersonBoundingBox();
    ASSERT_FALSE(roi.empty());
}

TEST_F(PersonDetectionShould, returnEmptyRoiIfEmptyMatReceived) {
    cv::Mat emptyMat;
    EXPECT_CALL(*cameraMock_, getFrame()).WillOnce(::testing::Return(emptyMat));
    ASSERT_TRUE(sut_->initialize());
    cv::Rect2d roi = sut_->getPersonBoundingBox();
    ASSERT_TRUE(roi.empty());
}

TEST_F(PersonDetectionShould, NotInitializeIfServerIsNotResponding) {
    EXPECT_CALL(*detector_, initialize()).WillOnce(::testing::Return(false));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(PersonDetectionShould, NotInitializeIfCameraIsNotResponding) {
    EXPECT_CALL(*cameraMock_, initialize()).WillOnce(::testing::Return(false));
    ASSERT_FALSE(sut_->initialize());
}

TEST_F(PersonDetectionShould, returnFalseWhileDeinitializingIfNotInitialized) {
    EXPECT_CALL(*cameraMock_, initialize()).WillOnce(::testing::Return(false));
    ASSERT_FALSE(sut_->initialize());
    ASSERT_FALSE(sut_->deinitialize());
}
