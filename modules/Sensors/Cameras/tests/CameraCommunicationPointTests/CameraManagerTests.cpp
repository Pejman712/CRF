/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */


#include <future>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Cameras/CameraMock.hpp"
#include "Cameras/CameraMockConfiguration.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;
using testing::AtLeast;
using testing::NiceMock;

using crf::sensors::cameras::CameraMock;
using crf::sensors::cameras::CameraMockConfiguration;
using crf::sensors::cameras::CameraManager;
using crf::sensors::cameras::Profile;

class CameraManagerShould : public ::testing::Test {
 protected:
    CameraManagerShould() :
        logger_("CameraManagerShould"),
        mock_(std::make_shared<NiceMock<CameraMockConfiguration>>()),
        manager_(std::make_unique<CameraManager>(mock_, std::chrono::seconds(3))) {
        logger_->info("{0} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    void SetUp() {
        mock_ = std::make_shared<NiceMock<CameraMockConfiguration>>();
        manager_ = std::make_unique<CameraManager>(mock_, std::chrono::seconds(3));
    }

    ~CameraManagerShould() {
        logger_->info("{0} END with {1}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::shared_ptr<NiceMock<CameraMockConfiguration>> mock_;
    std::unique_ptr<CameraManager> manager_;
};

TEST_F(CameraManagerShould, getFrameFailsIfNotRequestedBefore) {
    ASSERT_TRUE(manager_->getFrame(0).empty());
}

TEST_F(CameraManagerShould, getFrameFailsIfRequestedSocketIsNotActive) {
    ASSERT_TRUE(manager_->requestFrameStream(0, Profile(cv::Size(1280, 720), 30)));
    ASSERT_THROW(manager_->getFrame(1), std::out_of_range);
}

TEST_F(CameraManagerShould, wrongResolutionValues) {
    ASSERT_FALSE(manager_->requestFrameStream(0, Profile(cv::Size(0, 720), 30)));
    ASSERT_FALSE(manager_->requestFrameStream(0, Profile(cv::Size(1280, 0), 30)));
    ASSERT_FALSE(manager_->requestFrameStream(0, Profile(cv::Size(244, 4432), 30)));
}

TEST_F(CameraManagerShould, correctlyGetFrameAndUpdateResolution) {
    ASSERT_TRUE(manager_->requestFrameStream(0, Profile(cv::Size(1280, 720), 30)));
    cv::Mat mat;
    while (mat.empty())
        mat = manager_->getFrame(0, std::chrono::milliseconds(10000));

    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    mat = manager_->getFrame(0, std::chrono::milliseconds(10000));
    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    mat = manager_->getFrame(0, std::chrono::milliseconds(10000));
    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    mat = manager_->getFrame(0, std::chrono::milliseconds(10000));
    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    mat = manager_->getFrame(0, std::chrono::milliseconds(10000));
    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    mat = manager_->getFrame(0, std::chrono::milliseconds(10000));
    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    mat = manager_->getFrame(0, std::chrono::milliseconds(10000));
    ASSERT_EQ(mat.rows, 720);
    ASSERT_EQ(mat.cols, 1280);

    ASSERT_TRUE(manager_->requestFrameStream(0, Profile(cv::Size(640, 480), 30)));
    mat = manager_->getFrame(0, std::chrono::milliseconds(10000));

    ASSERT_EQ(mat.rows, 480);
    ASSERT_EQ(mat.cols, 640);
}

TEST_F(CameraManagerShould, requestTwoFramesAtTheSameTime) {
    manager_->requestFrameStream(0, Profile(cv::Size(1280, 720), 30));
    cv::Mat mat1;
    while (mat1.empty())
        mat1 = manager_->getFrame(0, std::chrono::milliseconds(10000));

    manager_->requestFrameStream(1, Profile(cv::Size(1920, 1080), 30));
    cv::Mat mat2;
    while (mat2.empty())
        mat2 = manager_->getFrame(1, std::chrono::milliseconds(10000));

    ASSERT_EQ(mat1.rows, 720);
    ASSERT_EQ(mat1.cols, 1280);

    ASSERT_EQ(mat2.rows, 1080);
    ASSERT_EQ(mat2.cols, 1920);
}

TEST_F(CameraManagerShould, returnsEmptyMatAfterTimeout) {
    manager_->requestFrameStream(0, Profile(cv::Size(1280, 720), 30));

    auto mat = manager_->getFrame(0, std::chrono::seconds(1));
    ASSERT_TRUE(mat.empty());
}

TEST_F(CameraManagerShould, correctlyGetSetParameters) {
    ASSERT_TRUE(manager_->requestFrameStream(0, Profile(cv::Size(1280, 720), 30)));
    auto parameters = manager_->getStatus();
    nlohmann::json property;
    property["brightness"] = 0;
    ASSERT_TRUE(manager_->setProperty(1, property));
}

TEST_F(CameraManagerShould, someParametersFailure) {
    nlohmann::json parameters;
    parameters["brightness"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["brightness"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["contrast"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["contrast"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["saturation"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["saturation"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["hue"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["hue"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["gain"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["gain"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["exposure"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["exposure"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["focus"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["focus"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["focus_mode"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["focus_mode"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["shutter"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["shutter"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["iso"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["iso"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["zoom"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["zoom"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["pan"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["pan"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["tilt"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["tilt"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));

    parameters["roll"] = "tmp";
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    parameters["roll"] = 1;
    ASSERT_TRUE(manager_->setProperty(1, parameters));
}

TEST_F(CameraManagerShould, failsToSetParametersIfCameraDoesNotSupportIt) {
    nlohmann::json parameters;
    parameters["roll"] = 1;
    mock_->can_set_property_ = false;
    ASSERT_FALSE(manager_->setProperty(1, parameters));
    mock_->can_set_property_ = true;
    ASSERT_TRUE(manager_->setProperty(1, parameters));
}

TEST_F(CameraManagerShould, parametersMissingFieldIfOperationNotPermitted) {
    ASSERT_TRUE(manager_->requestFrameStream(0, Profile(cv::Size(1280, 720), 30)));
    mock_->can_set_property_ = false;
    nlohmann::json parameters = manager_->getStatus();
    crf::expected<int> bright = parameters["property"]["brightness"].get<crf::expected<int>>();
    ASSERT_FALSE(bright);
}
