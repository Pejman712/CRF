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
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/depth.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "CommonInterfaces/IInitializable.hpp"
#include "Cameras/CameraClient/CameraClient.hpp"
#include "RGBDCameras/IRGBDCamera.hpp"
#include "CommunicationUtility/StreamReader.hpp"
#include "VisionUtility/Image/ImageJSONConverter.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

using crf::sensors::cameras::CameraClient;
using crf::sensors::cameras::Profile;
using crf::sensors::cameras::Property;

/**
 * @ingroup group_rgbdcamera_client
 * @brief Client for the RGBD cameras, it extends the already exitsting
 * camera client and expands it.
 *
 * @details This class inherits from CameraClient
 *
 * @{
 */
class RGBDCameraClient : public IRGBDCamera, CameraClient {
 public:
    RGBDCameraClient() = delete;

    /**
     * @brief Construct a new RGBDCameraClient object
     *
     * @param socket Socket to send the packets
     * @param serverReplyTimeout Timeout for server reply
     * @param frequency Frequency of the status streamer
     * @param priority Priority of the client
     */
    explicit RGBDCameraClient(
        std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
        const std::chrono::milliseconds& serverReplyTimeout,
        const float& frequency,
        const uint32_t& priority);
    explicit RGBDCameraClient(const CameraClient&) = delete;
    explicit RGBDCameraClient(CameraClient&&) = delete;
    ~RGBDCameraClient() override;

    /**
     * @brief Initializes the communicaion with the server
     *
     */
    bool initialize() override;

    /**
     * @brief Deinitializes the clietn and cuts the communicaion with the server
     *
     */
    bool deinitialize() override;

    /**
     * @brief Get an RGB image
     *
     * @return cv::Mat Image returned
     */
    cv::Mat captureImage() override;

    /**
     * @brief Get an RGBD image
     *
     * @return cv::rgbd::RgbdFrame RGBD image returned
     */
    cv::rgbd::RgbdFrame captureImageAndDepth() override;

    /**
     * @brief Get a point cloud
     *
     * @return RGBDPointCloud Point cloud returned
     */
    RGBDPointCloud capturePointCloud() override;

    /**
     * @brief Set the image profile of the camera
     *
     * @param profile Profile to set
     * @return true If successful
     * @return false Otherwise
     */
    bool setProfile(const Profile& profile) override;

    /**
     * @brief Set the image and depth profiles
     *
     * @param streamprofile Image profile
     * @param depthprofile Depth profile
     * @return true If successful
     * @return false Otherwise
     */
    bool setProfile(const Profile& streamprofile, const Profile& depthprofile) override;

    /**
     * @brief Set the Depth Profile
     *
     * @param profile depth profile to set
     * @return true If successful
     * @return false Otherwise
     */
    bool setDepthProfile(const Profile& profile) override;

    /**
     * @brief Get the current image profile
     *
     * @return crf::expected<Profile> Current profile
     */
    crf::expected<Profile> getProfile() override;

    /**
     * @brief Get the current depth profile
     *
     * @return crf::expected<Profile> Current depth profile
     */
    crf::expected<Profile> getDepthProfile() override;

    /**
     * @brief List all available image profiles
     *
     * @return std::vector<Profile> List of image profiles
     */
    std::vector<Profile> listProfiles() override;

    /**
     * @brief List all available depth profiles
     *
     * @return std::vector<Profile> List of depth profiles
     */
    std::vector<Profile> listDepthProfiles() override;

    /**
     * @brief Set a property of the camera
     *
     * @param property Property to set
     * @param value Desired value
     * @return crf::expected<bool> Result
     */
    crf::expected<bool> setProperty(const Property& property, const int& value) override;

    /**
     * @brief Get the value of a property
     *
     * @param property Desired property
     * @return crf::expected<int> Returned value
     */
    crf::expected<int> getProperty(const Property& property) override;

    /**
     * @brief Get the Color Camera Matrix
     *
     * @return crf::expected<cv::Mat> Matrix returned
     */
    crf::expected<cv::Mat> getColorCameraMatrix() override;

    /**
     * @brief Get the Color Distortion Matrix
     *
     * @return crf::expected<cv::Mat> Matrix returned
     */
    crf::expected<cv::Mat> getColorDistortionMatrix() override;

    /**
     * @brief Get the Depth Camera Matrix
     *
     * @return crf::expected<cv::Mat> Matrix returned
     */
    crf::expected<cv::Mat> getDepthCameraMatrix() override;

    /**
     * @brief Get the Depth Distortion Matrix
     *
     * @return crf::expected<cv::Mat> Matrix returned
     */
    crf::expected<cv::Mat> getDepthDistortionMatrix() override;

    /**
     * @brief Get the Depth2 Color Extrinsics
     *
     * @return crf::expected<cv::Mat> Matrix returned
     */
    crf::expected<cv::Mat> getDepth2ColorExtrinsics() override;

 protected:
    crf::expected<cv::Mat> colorCameraMatrix_;
    crf::expected<cv::Mat> colorDistortionMatrix_;
    crf::expected<cv::Mat> depthCameraMatrix_;
    crf::expected<cv::Mat> depthDistortionMatrix_;
    crf::expected<cv::Mat> extrinsic_;

    std::vector<Profile> depthProfiles_;
    crf::expected<Profile> currentDepthProfile_;
    crf::expected<Profile> currentImageProfile_;

    cv::Mat depthImage_;

    utility::logger::EventLogger logger_;

    /**
     * @brief Method to start the frame stream
     *
     * @param streamprofile Requested image profile
     * @param depthprofile Requested depth profile
     * @param pointcloud If pointclouds are desired
     * @return true If successful
     * @return false Otherwise
     */
    bool startFrameStream(
        const Profile& streamprofile, const Profile& depthprofile, const bool& pointcloud);

    /**
     * @brief Method to parse the status received by the server
     *
     * @param json Status received
     */
    void parseStatus(const nlohmann::json& json) override;

    /**
     * @brief Method to parse the RGBD packet received
     *
     * @param buffer Buffer with the bytes received
     */
    void parseRGBDFramePacket(const std::string& buffer);
};

/**@}*/

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
