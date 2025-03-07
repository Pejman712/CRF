/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <condition_variable>
#include <functional>
#include <map>
#include <vector>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

#include "RGBDCameras/IRGBDCamera.hpp"
#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

/**
 * @ingroup group_rgbdcamera_communication_point
 * @brief RGBD Camera manager class for camera communication point. The camera manager class
 *        ensures thread safe access to the camera. It is also responsible to optimize the
 *        resources. If no communication point is requesting frames the camera is deinitialized to
 *        release resources on the running controller (e.g. USB bandwidth). The communication point
 *        requests a frame stream to the camera manager with a certain resolution. It is also
 *        responsible of changing the camera parameters.
 *
 * @details This class inherits from CameraManager
 *
 * @{
 */
class RGBDCameraManager: public cameras::CameraManager {
 public:
    RGBDCameraManager() = delete;

    /**
     *
     * @brief Construct a new RGBDCameraManager object
     *
     * @param camera Camera to manage
     * @param initializationTimeout Timeout until the object deinitialzes if not used
     * @param controlAccessTimeout Timeout until the control is released if not used
     */
    RGBDCameraManager(std::shared_ptr<IRGBDCamera> camera,
        const std::chrono::milliseconds& initializationTimeout = std::chrono::seconds(20),
        const std::chrono::milliseconds& controlAccessTimeout = std::chrono::seconds(10));
    RGBDCameraManager(const RGBDCameraManager& other) = delete;
    RGBDCameraManager(RGBDCameraManager&& other) = delete;
    ~RGBDCameraManager();

    /**
     * @brief Get the Status of the RGBD camera
     *
     * @return nlohmann::json Status returned
     */
    nlohmann::json getStatus() override;

    /**
     * @brief Get the image frame of the camera RGB
     *
     * @param stream_id ID of the stream
     * @param timeout Timeout waiting for the frame
     * @return cv::Mat Image requested
     */
    cv::Mat getFrame(
        int stream_id,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(500)) override;

    /**
     * @brief Get the image frame with depth RGBD
     *
     * @param stream_id Id of the stream
     * @param timeout Timeout waiting for the frame
     * @return cv::rgbd::RgbdFrame Frame with depth requested
     */
    cv::rgbd::RgbdFrame getRGBDFrame(
        int stream_id,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(500));

    /**
     * @brief Get the Point Cloud object
     *
     * @param stream_id Id of the stream
     * @param timeout Timeout waiting for the frame
     * @return pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Point cloud requested
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud(
        int stream_id,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(500));

    /**
     * @brief Get the RGBD frame and the pointcloud
     *
     * @param stream_id Id of the stream
     * @param timeout Timeout waiting for the frame
     * @return RGBDPointCloud Frame an pointcloud requested
     */
    RGBDPointCloud getRGBDFrameAndPointCloud(
        int stream_id,
        const std::chrono::milliseconds& timeout = std::chrono::milliseconds(500));

    /**
     * @brief Method to request a new frame stream of RGB images
     *
     * @param stream_id Id of the new stream requested
     * @param profile Profile for the stream requested
     * @return true If successful
     * @return false Otherwise
     */
    bool requestFrameStream(int stream_id, const Profile& profile) override;

    /**
     * @brief Method to request a new frame stream of RGBD and pointclouds
     *
     * @param stream_id Id of the new stream requested
     * @param streamProfile Image profile for the stream requested
     * @param depthProfile Depth profile for the stream requested
     * @param pointcloud_stream If point clouds are to be sent or not
     * @return true If successful
     * @return false Otherwise
     */
    bool requestFrameStream(
        int stream_id, const Profile& streamProfile, const Profile& depthProfile,
        bool pointcloud_stream);

    /**
     * @brief Remove a request of a frame stream
     *
     * @param stream_id ID of the stream to remove
     */
    void removeFrameStream(int stream_id) override;

 protected:
    /**
     * @brief Struct to group the requested streams and their values
     *
     */
    struct RGBDStreamRequest {
        bool depthRequested;
        bool pointCloudRequested;
        cv::Size imageResolution;
        uint64_t imageFramerate;
        cv::Size depthResolution;
        uint64_t depthFramerate;
    };

    RGBDPointCloud latestPointCloud_;

    std::vector<crf::sensors::cameras::Profile> depthProfiles_;
    Profile currentDepthProfile_;

    std::map<int, RGBDStreamRequest> requestedRGBDStreams_;

    std::atomic<bool> depthRequested_;
    std::atomic<bool> pointCloudRequested_;

    /**
     * @brief Frame grabber to take the frame from the camera and save it
     *
     */
    void frameGrabber() override;

    /**
     * @brief Meethod to check the requested profiles and select the most demanding one
     *
     * @return true If successful
     * @return false Otherwise
     */
    bool updateProfile() override;

    /**
     * @brief Get the Max Requested Depth Resolution
     *
     * @return cv::Size Max resolution
     */
    cv::Size getMaxRequestedDepthResolution();

    /**
     * @brief Get the Max Requested Depth Framerate
     *
     * @return int Max depth framerate
     */
    int getMaxRequestedDepthFramerate();

    /**
     * @brief Check all requests and see if depth is requested by any
     *
     * @return true If successful
     * @return false Otherwise
     */
    bool isDepthRequested();

    /**
     * @brief Check all requests and see if point clouds are requested by any
     *
     * @return true If successful
     * @return false Otherwise
     */
    bool isPointCloudRequested();

 private:
    std::shared_ptr<IRGBDCamera> rgbdCamera_;
    utility::logger::EventLogger logger_;
};

/**@}*/

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
