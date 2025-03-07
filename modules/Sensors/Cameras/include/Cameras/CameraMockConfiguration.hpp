/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Álvaro García González CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <atomic>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <opencv2/opencv.hpp>

#include "Cameras/CameraMock.hpp"

using crf::sensors::cameras::Property;

namespace crf::sensors::cameras {

class CameraMockConfiguration: public CameraMock {
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

 public:
    bool can_initialize_;
    bool can_deinitialize_;
    bool can_capture_image_;
    bool can_set_profile_;
    bool can_get_profile_;
    bool can_set_property_;
    bool can_get_property_;

    CameraMockConfiguration():
        can_initialize_(true),
        can_deinitialize_(true),
        can_capture_image_(true),
        can_set_profile_(true),
        can_get_profile_(true),
        can_set_property_(true),
        can_get_property_(true),
        initialized_(false),
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

}  // namespace crf::sensors::cameras
