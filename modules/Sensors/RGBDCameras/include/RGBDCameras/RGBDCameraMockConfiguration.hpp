/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <utility>
#include "RGBDCameras/RGBDCameraMock.hpp"

using ::testing::NiceMock;

using crf::sensors::cameras::Profile;

namespace crf {
namespace sensors {
namespace rgbdcameras {

class RGBDCameraMockConfiguration: public RGBDCameraMock {
 protected:
    const std::vector<uint64_t> listFramerates_ = {5, 10, 15, 20, 25, 30};
    const std::vector<Profile> listProfiles_ = {
        Profile(cv::Size(100, 100), listFramerates_),
        Profile(cv::Size(150, 150), listFramerates_),
        Profile(cv::Size(640, 480), listFramerates_),
        Profile(cv::Size(800, 600), listFramerates_),
        Profile(cv::Size(1280, 720), listFramerates_),
        Profile(cv::Size(1920, 1080), listFramerates_)
    };

    const std::vector<uint64_t> listDepthFramerates_ = { 5, 10, 15, 20, 25, 30 };
    const std::vector<Profile> listDepthProfiles_ = {
        Profile(cv::Size(100, 100), listDepthFramerates_),
        Profile(cv::Size(150, 150), listDepthFramerates_),
        Profile(cv::Size(640, 480), listDepthFramerates_),
        Profile(cv::Size(800, 600), listDepthFramerates_),
        Profile(cv::Size(1280, 720), listDepthFramerates_),
        Profile(cv::Size(1920, 1080), listDepthFramerates_)
    };

 public:
    bool can_initialize_;
    bool can_deinitialize_;
    bool can_capture_image_;
    bool can_set_profile_;
    bool can_get_profile_;
    bool can_set_property_;
    bool can_get_property_;

 public:
    RGBDCameraMockConfiguration():
        can_initialize_(true),
        can_deinitialize_(true),
        can_capture_image_(true),
        can_set_profile_(true),
        can_get_profile_(true),
        can_set_property_(true),
        can_get_property_(true),
        initialized_(false),
        depthProfile_(cv::Size(640, 480), 10),
        profile_(cv::Size(640, 480), 10),
        brightness_(0),
        contrast_(0),
        saturation_(0),
        hue_(0),
        gain_(0),
        exposure_(0),
        focus_(0),
        focusmode_(0),
        shutter_(0),
        iso_(0),
        zoom_(0),
        pan_(0),
        tilt_(0),
        roll_(0),
        frame_set_(false) {
        ON_CALL(*this, initialize()).WillByDefault(testing::Invoke(
            [this]() {
                if (!can_initialize_) return false;
                initialized_ = true;
                return true;
            }));

        ON_CALL(*this, deinitialize()).WillByDefault(testing::Invoke(
            [this]() {
                if (!can_deinitialize_) return false;
                initialized_ = false;
                return true;
            }));

        ON_CALL(*this, captureImage()).WillByDefault(testing::Invoke(
            [this]() {
                if (!initialized_) return cv::Mat();
                if (!can_capture_image_) return cv::Mat();
                if (frame_set_) return frame_;
                auto mat = cv::Mat(profile_.resolution, CV_8UC3);
                cv::randu(mat, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
                return mat;
            }));

        ON_CALL(*this, setProfile(testing::_)).WillByDefault(testing::Invoke(
            [this](const Profile& profile) {
                if (!initialized_) return false;
                if (!can_set_profile_) return false;
                for (uint64_t i = 0; i < listProfiles_.size(); i++) {
                    if (listProfiles_[i].resolution != profile.resolution) continue;
                    for (uint64_t j = 0; j < listProfiles_[i].framerates.size(); j++) {
                        if (listProfiles_[i].framerates[j] != profile.framerate) continue;
                        profile_ = profile;
                        return true;
                    }
                }
                return false;
            }));

        ON_CALL(*this, getProfile()).WillByDefault(testing::Invoke(
            [this]() -> crf::expected<Profile> {
                if (!initialized_) return crf::Code::NotInitialized;
                if (!can_get_profile_) return crf::Code::ThirdPartyQueryFailed;
                return profile_;
            }));

        ON_CALL(*this, captureImageAndDepth()).WillByDefault(testing::Invoke(
            [this] () {
                if (!initialized_) return cv::rgbd::RgbdFrame();
                if (!can_capture_image_) return cv::rgbd::RgbdFrame();
                cv::rgbd::RgbdFrame frame;
                frame.image = cv::Mat(profile_.resolution, CV_8UC3);
                cv::randu(frame.image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

                frame.depth = cv::Mat(depthProfile_.resolution, CV_16U);
                cv::randu(frame.depth, cv::Scalar(0), cv::Scalar(2000));
                return frame;
            }));

        ON_CALL(*this, capturePointCloud()).WillByDefault(testing::Invoke(
            [this] () {
                cv::rgbd::RgbdFrame frame;
                RGBDPointCloud rgbdCloud;
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
                    new pcl::PointCloud<pcl::PointXYZRGBA>());
                if (!initialized_) return rgbdCloud;
                if (!can_capture_image_) return rgbdCloud;
                frame.image = cv::Mat(profile_.resolution, CV_8UC3);
                cv::randu(frame.image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

                frame.depth = cv::Mat(depthProfile_.resolution, CV_16U);
                cv::randu(frame.depth, cv::Scalar(0), cv::Scalar(2000));

                cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>(
                    frame.depth.cols, frame.depth.rows);
                for (int dy = 0; dy < frame.depth.rows; ++dy) {
                    for (int dx = 0; dx < frame.depth.cols; ++dx) {
                        cloud->at(dx, dy).r = frame.image.at<cv::Vec3b>(dy, dx)[2];
                        cloud->at(dx, dy).g = frame.image.at<cv::Vec3b>(dy, dx)[1];
                        cloud->at(dx, dy).b = frame.image.at<cv::Vec3b>(dy, dx)[0];

                        cloud->at(dx, dy).x = 0;
                        cloud->at(dx, dy).y = 0;
                        cloud->at(dx, dy).z = 0;
                        cloud->at(dx, dy).a = 255;
                    }
                }
                rgbdCloud.pointcloud = cloud;
                rgbdCloud.frame = frame;
                return rgbdCloud;
            }));

        ON_CALL(*this, setProfile(testing::_, testing::_)).WillByDefault(testing::Invoke(
            [this] (const Profile& imgProfile, const Profile& depthProfile) {
                if (!initialized_) return false;
                if (!can_set_profile_) return false;
                if (!this->setProfile(imgProfile)) return false;
                if (!this->setDepthProfile(depthProfile)) return false;
                return true;
            }));

        ON_CALL(*this, setDepthProfile(testing::_)).WillByDefault(testing::Invoke(
            [this] (const Profile& profile) {
                if (!initialized_) return false;
                if (!can_set_profile_) return false;
                for (uint64_t i = 0; i < listDepthProfiles_.size(); i++) {
                    if (listDepthProfiles_[i].resolution != profile.resolution) continue;
                    for (uint64_t j = 0; j < listDepthProfiles_[i].framerates.size(); j++) {
                        if (listDepthProfiles_[i].framerates[j] != profile.framerate) continue;
                        depthProfile_ = profile;
                        return true;
                    }
                }
                return true;
            }));

        ON_CALL(*this, setProperty(testing::_, testing::_)).WillByDefault(testing::Invoke(
            [this](const Property& property, const int& value) -> crf::expected<bool> {
                if (!initialized_) return crf::Code::NotInitialized;
                if (!can_set_property_) return crf::Code::ThirdPartyQueryFailed;
                if (property == Property::BRIGHTNESS)
                    brightness_ = value;
                else if (property == Property::CONTRAST)
                    contrast_ = value;
                else if (property == Property::SATURATION)
                    saturation_ = value;
                else if (property == Property::HUE)
                    hue_ = value;
                else if (property == Property::GAIN)
                    gain_ = value;
                else if (property == Property::EXPOSURE)
                    exposure_ = value;
                else if (property == Property::FOCUS)
                    focus_ = value;
                else if (property == Property::FOCUSMODE)
                    focusmode_ = value;
                else if (property == Property::SHUTTER)
                    shutter_ = value;
                else if (property == Property::ISO)
                    iso_ = value;
                else if (property == Property::ZOOM)
                    brightness_ = value;
                else if (property == Property::PAN)
                    pan_ = value;
                else if (property == Property::TILT)
                    tilt_ = value;
                else if (property == Property::ROLL)
                    roll_ = value;
                else
                    return crf::Code::BadRequest;
                return true;
            }));

        ON_CALL(*this, getProperty(testing::_)).WillByDefault(testing::Invoke(
            [this](const Property& property) -> crf::expected<int> {
                if (!initialized_) return crf::Code::NotInitialized;
                if (!can_set_property_) return crf::Code::ThirdPartyQueryFailed;
                if (property == Property::BRIGHTNESS)
                    return brightness_;
                else if (property == Property::CONTRAST)
                    return contrast_;
                else if (property == Property::SATURATION)
                    return saturation_;
                else if (property == Property::HUE)
                    return hue_;
                else if (property == Property::GAIN)
                    return gain_;
                else if (property == Property::EXPOSURE)
                    return exposure_;
                else if (property == Property::FOCUS)
                    return focus_;
                else if (property == Property::FOCUSMODE)
                    return focusmode_;
                else if (property == Property::SHUTTER)
                    return shutter_;
                else if (property == Property::ISO)
                    return iso_;
                else if (property == Property::ZOOM)
                    return brightness_;
                else if (property == Property::PAN)
                    return pan_;
                else if (property == Property::TILT)
                    return tilt_;
                else if (property == Property::ROLL)
                    return roll_;
                else
                    return crf::Code::BadRequest;
            }));

        ON_CALL(*this, listProfiles()).WillByDefault(testing::Invoke(
            [this]() {
                return listProfiles_;
            }));

        ON_CALL(*this, listDepthProfiles()).WillByDefault(testing::Invoke(
            [this] () -> std::vector<Profile> {
                return listDepthProfiles_;
            }));

        ON_CALL(*this, getColorCameraMatrix()).WillByDefault(testing::Invoke(
            [this] () -> crf::expected<cv::Mat> {
                if (!initialized_) return crf::Code::NotInitialized;
                cv::Mat intrinsics;
                intrinsics = cv::Mat::eye(3, 3, CV_32F);
                intrinsics.at<float>(0, 0) = 1;
                intrinsics.at<float>(1, 1) = 1;
                intrinsics.at<float>(0, 2) = profile_.resolution.width/2;
                intrinsics.at<float>(1, 2) = profile_.resolution.height/2;
                return intrinsics;
            }));

        ON_CALL(*this, getColorDistortionMatrix()).WillByDefault(testing::Invoke(
            [this] () -> crf::expected<cv::Mat> {
                if (!initialized_) return crf::Code::NotInitialized;
                return cv::Mat::zeros(1, 5, CV_32F);
            }));

        ON_CALL(*this, getDepthCameraMatrix()).WillByDefault(testing::Invoke(
            [this] () -> crf::expected<cv::Mat> {
                if (!initialized_) return crf::Code::NotInitialized;
                cv::Mat intrinsics;
                intrinsics = cv::Mat::eye(3, 3, CV_32F);
                intrinsics.at<float>(0, 0) = 1;
                intrinsics.at<float>(1, 1) = 1;
                intrinsics.at<float>(0, 2) = depthProfile_.resolution.width/2;
                intrinsics.at<float>(1, 2) = depthProfile_.resolution.height/2;
                return intrinsics;
            }));

        ON_CALL(*this, getDepthDistortionMatrix()).WillByDefault(testing::Invoke(
           [this] () -> crf::expected<cv::Mat> {
                if (!initialized_) return crf::Code::NotInitialized;
                return cv::Mat::zeros(1, 5, CV_32F);;
            }));

        ON_CALL(*this, getDepth2ColorExtrinsics()).WillByDefault(testing::Invoke(
           [this] () -> crf::expected<cv::Mat> {
                if (!initialized_) return crf::Code::NotInitialized;
                return cv::Mat::eye(4, 4, CV_32F);
            }));
    }

    void setFrame(const cv::Mat& frame) {
        frame_ = frame;
        frame_set_ = true;
        profile_.resolution = frame.size();
    }

    void removeFrame() {
        frame_set_ = false;
    }

 protected:
    Profile depthProfile_;
    Profile profile_;
    bool initialized_;

    int brightness_;
    int contrast_;
    int saturation_;
    int hue_;
    int gain_;
    int exposure_;
    int focus_;
    int focusmode_;
    int shutter_;
    int iso_;
    int zoom_;
    int pan_;
    int tilt_;
    int roll_;

    std::atomic<bool> frame_set_;
    cv::Mat frame_;
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
