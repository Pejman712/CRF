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
#include <opencv2/rgbd/depth.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Cameras/ICamera.hpp"
#include "RGBDCameras/RGBDPointCloud.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

using crf::sensors::cameras::ICamera;
using crf::sensors::cameras::Profile;

/**
 * @ingroup group_rgbdcamera
 * @brief Abstract interface for the RGBD cameras. It acts as an extension of the
 * existing cameras interface
 *
 * @details This class inherits from ICamera
 *
 * @{
 */
class IRGBDCamera : public ICamera {
 public:
    virtual ~IRGBDCamera() = default;

    /**
     * @brief Get a single image encoded as a cv::Mat
     *        and the corresponding depth frame as a cv::Mat
     *
     * @return cv::rgbd::RgbdFrame containing the RGB and depth frames
     */
    virtual cv::rgbd::RgbdFrame captureImageAndDepth() = 0;

    /**
     * @brief Get a color image encoded,
     * it is corresponding depth frame encoded as a cv::Mat
     * and the PointCloud created usign both
     *
     * @return Pair with a pcl::pointcloud with the pointcloud
     * and a cv::rgbd::RgbdFrame containing the RGB and depth frames
     */
    virtual RGBDPointCloud capturePointCloud() = 0;

    using ICamera::setProfile;

    /**
     * @brief Set the color and depth video profile
     *
     * @param resolution
     * @param framerate
     * @param depth_resolution
     * @param depth_framerate
     * @return true
     * @return false
     */
    virtual bool setProfile(const Profile& profile, const Profile& depthProfile) = 0;

    /**
     * @brief Set the depth video profile
     *
     * @param resolution
     * @param framerate
     * @return true
     * @return false
     */
    virtual bool setDepthProfile(const Profile& profile) = 0;

    /**
     * @brief Get the depth video profile
     *
     * @param resolution
     * @param framerate
     * @return true
     * @return false
     */
    virtual crf::expected<Profile> getDepthProfile() = 0;

    /**
     * @brief Lists all available profiles
     * @return std::vector<Profile> all available profiles
     */
    virtual std::vector<Profile> listDepthProfiles() = 0;

    /**
     * @brief Get the camera matrix for the color camera
     *
     * @return std::optional<cv::Mat> containing the camera matrix if available, empty otherwise
     */
    virtual crf::expected<cv::Mat> getColorCameraMatrix() = 0;

    /**
     * @brief Get the distortion matrix for the color camera
     *
     * @return std::optional<cv::Mat> containing the distortion matrix if available, empty otherwise
     */
    virtual crf::expected<cv::Mat> getColorDistortionMatrix() = 0;

    /**
     * @brief Get the camera matrix for the depth camera
     *
     * @return std::optional<cv::Mat> containing the camera matrix if available, empty otherwise
     */
    virtual crf::expected<cv::Mat> getDepthCameraMatrix() = 0;

    /**
     * @brief Get the distortion matrix for the depth camera
     *
     * @return std::optional<cv::Mat> containing the distortion matrix if available, empty otherwise
     */
    virtual crf::expected<cv::Mat> getDepthDistortionMatrix() = 0;

    /**
     * @brief Get the extrinsic matrix from the depth to the color camera
     *
     * @return std::optional<cv::Mat> containing the extrinsic matrix if available, empty otherwise
     */
    virtual crf::expected<cv::Mat> getDepth2ColorExtrinsics() = 0;
};

/**@}*/

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
