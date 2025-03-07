/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alvaro Garcia Gonzalez BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "crf/expected.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "Cameras/CameraProperty.hpp"
#include "Cameras/CameraProfile.hpp"

namespace crf::sensors::cameras {

/**
 * @ingroup group_cameras
 * @brief Abstract interface for all the cameras. It includes pure virtual methods to
 * implement in all cameras.
 *
 * @{
 */

class ICamera : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~ICamera() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Get a single image encoded as a cv::Mat
     *
     * @return cv::Mat containing the RGB frame
     */
    virtual cv::Mat captureImage() = 0;

    /**
     * @brief Set the video profile
     *
     * @param resolution
     * @param fps
     * @return true
     * @return false
     */
    virtual bool setProfile(const Profile& profile) = 0;

    /**
     * @brief Get the video profile
     *
     * @param resolution
     * @param fps
     * @return true
     * @return false
     */
    virtual crf::expected<Profile> getProfile() = 0;

    /**
     * @brief Lists all available profiles
     * @return std::vector<Profile> all available profiles
     */
    virtual std::vector<Profile> listProfiles() = 0;

    /**
     * @brief Set a camera property
     * @param property: Enum that specifies the property.
     * @param value: Value of the property.
     * @return true: The property was tried to be set by the driver. Does not guarantee that the parameter was set.
     * @return false: The property was not set.
     */
    virtual crf::expected<bool> setProperty(const Property& property, const int& value) = 0;

    /**
     * @brief Get a camera property
     * @param property: Enum that specifies the property.
     * @return T: The value of the property if it could be set
     */
    virtual crf::expected<int> getProperty(const Property& property) = 0;
};

/**@}*/

}  // namespace crf::sensors::cameras
