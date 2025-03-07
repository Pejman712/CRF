/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "VisionUtility/Image/ImageJSONConverter.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

RGBDCameraManager::RGBDCameraManager(std::shared_ptr<IRGBDCamera> camera,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout):
    CameraManager(camera, initializationTimeout, controlAccessTimeout),
    depthRequested_(false),
    pointCloudRequested_(false),
    rgbdCamera_(camera),
    logger_("RGBDCameraManager") {
        logger_->debug("CTor");

        currentProfile_.framerates = {0};
        currentDepthProfile_.framerates = {0};

        depthProfiles_ = rgbdCamera_->listDepthProfiles();
}

RGBDCameraManager::~RGBDCameraManager() {
    logger_->debug("DTor");
}

nlohmann::json RGBDCameraManager::getStatus() {
    logger_->debug("getStatus()");
    nlohmann::json statusJSON = CameraManager::getStatus();
    crf::expected<cv::Mat> matrix;
    std::unique_lock<std::mutex> lock(cameraMtx_);
    statusJSON["color_intrinsic"] = rgbdCamera_->getColorCameraMatrix();
    statusJSON["color_distortion"] = rgbdCamera_->getColorDistortionMatrix();
    statusJSON["depth_intrinsic"] = rgbdCamera_->getDepthCameraMatrix();
    statusJSON["depth_distortion"] = rgbdCamera_->getDepthDistortionMatrix();
    statusJSON["extrinsic"] = rgbdCamera_->getDepth2ColorExtrinsics();
    statusJSON["current_depth_profile"]= rgbdCamera_->getDepthProfile();
    lock.unlock();
    statusJSON["available_depth_profiles"] = depthProfiles_;
    return statusJSON;
}

cv::Mat RGBDCameraManager::getFrame(int stream_id, const std::chrono::milliseconds& timeout) {
    if (stopGrabber_) {
        logger_->error("getFrame(): Frame grabber is not running!");
        return cv::Mat();
    }
    cv::Size resolution;
    cv::Mat frame;

    {
        std::shared_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
        resolution = requestedRGBDStreams_.at(stream_id).imageResolution;
    }
    {
        std::shared_lock<std::shared_timed_mutex> lock(frameMtx_);
        frame = latestPointCloud_.frame.image;
    }
    if (frame.empty() || resolution.empty()) return frame;
    cv::resize(frame, frame, resolution);
    return frame;
}

cv::rgbd::RgbdFrame RGBDCameraManager::getRGBDFrame(
    int stream_id, const std::chrono::milliseconds& timeout) {
    if (stopGrabber_) {
        logger_->error("getRGBDFrame(): Frame grabber is not running!");
        return cv::rgbd::RgbdFrame();
    }
    cv::Size imageResolution;
    cv::Size depthResolution;
    cv::rgbd::RgbdFrame frame;

    {
        std::shared_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
        imageResolution = requestedRGBDStreams_.at(stream_id).imageResolution;
        depthResolution = requestedRGBDStreams_.at(stream_id).depthResolution;
    }

    {
        std::shared_lock<std::shared_timed_mutex> lock(frameMtx_);
        frame = latestPointCloud_.frame;
    }
    if (frame.image.empty() || frame.depth.empty()) return frame;
    if (imageResolution.empty() || depthResolution.empty()) return frame;
    cv::resize(frame.image, frame.image, imageResolution);
    cv::resize(frame.depth, frame.depth, depthResolution);
    return frame;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RGBDCameraManager::getPointCloud(
    int stream_id, const std::chrono::milliseconds& timeout) {
    if (stopGrabber_) {
        logger_->error("getPointCloud(): Frame grabber is not running!");
        return pcl::PointCloud<pcl::PointXYZRGBA>::Ptr();
    }

    std::shared_lock<std::shared_timed_mutex> lock(frameMtx_);
    return latestPointCloud_.pointcloud;
}

RGBDPointCloud RGBDCameraManager::getRGBDFrameAndPointCloud(
    int stream_id, const std::chrono::milliseconds& timeout) {
    if (stopGrabber_) {
        logger_->error("getRGBDFrameAndPointCloud(): Frame grabber is not running!");
        RGBDPointCloud empty;
        return empty;
    }
    cv::Size imageResolution;
    cv::Size depthResolution;
    RGBDPointCloud rgbd;

    {
        std::shared_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
        imageResolution = requestedRGBDStreams_.at(stream_id).imageResolution;
        depthResolution = requestedRGBDStreams_.at(stream_id).depthResolution;
    }

    {
        std::shared_lock<std::shared_timed_mutex> lock(frameMtx_);
        rgbd = latestPointCloud_;
    }
    if (rgbd.frame.image.empty() || rgbd.frame.depth.empty()) return rgbd;
    if (imageResolution.empty() || depthResolution.empty()) return rgbd;
    cv::resize(rgbd.frame.image, rgbd.frame.image, imageResolution);
    cv::resize(rgbd.frame.depth, rgbd.frame.depth, depthResolution);
    return rgbd;
}

bool RGBDCameraManager::requestFrameStream(int stream_id, const Profile& profile) {
    logger_->debug("requestFrameStream(profile)");
    auto it = std::find_if(cameraProfiles_.begin(), cameraProfiles_.end(),
        [profile](Profile p){
            return p.resolution == profile.resolution;
        });
    if (it == cameraProfiles_.end()) {
        logger_->error("requestFrameStream(): Resolution not valid");
        return false;
    }
    uint64_t framerate = profile.framerate;
    if (std::find(it->framerates.begin(), it->framerates.end(), profile.framerate) == it->framerates.end()) {  // NOLINT
        uint64_t maxFramerate = *std::max_element(it->framerates.begin(), it->framerates.end());
        logger_->error("requestFrameStream(): Framerate ({}) not valid for this resolution, using "
            "the max ({})", profile.framerate, maxFramerate);
        framerate = maxFramerate;
    }

    RGBDStreamRequest request;
    request.depthRequested = false;
    request.pointCloudRequested = false;
    request.imageResolution = profile.resolution;
    request.imageFramerate = framerate;

    std::unique_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    requestedRGBDStreams_[stream_id] = request;
    lock.unlock();
    updateProfile();
    sleepGrabber_ = false;
    startFrameGrabber();
    return true;
}

bool RGBDCameraManager::requestFrameStream(
    int stream_id, const Profile& streamProfile, const Profile& depthProfile,
    bool pointcloud_stream) {
    logger_->debug("startFrameStream(profile, profile, pc)");
    auto it = std::find_if(cameraProfiles_.begin(), cameraProfiles_.end(),
        [streamProfile](Profile p){
            return p.resolution == streamProfile.resolution;
        });
    if (it == cameraProfiles_.end()) {
        logger_->error("startFrameStream(): Resolution not valid");
        return false;
    }
    if (std::find(it->framerates.begin(), it->framerates.end(), streamProfile.framerates[0]) == it->framerates.end()) {  // NOLINT
        logger_->error("startFrameStream(): Framerate not valid for this resolution");
        return false;
    }

    // Check depth profile
    it = std::find_if(depthProfiles_.begin(), depthProfiles_.end(),
        [depthProfile](Profile p){
            return p.resolution == depthProfile.resolution;
        });
    if (it == depthProfiles_.end()) {
        logger_->error("startFrameStream(): Depth resolution not valid");
        return false;
    }
    if (std::find(it->framerates.begin(), it->framerates.end(), depthProfile.framerates[0]) == it->framerates.end()) {   // NOLINT
        logger_->error("startFrameStream(): Depth framerate not valid for this depth resolution");
        return false;
    }

    RGBDStreamRequest request;
    request.imageResolution = streamProfile.resolution;
    request.imageFramerate = streamProfile.framerates[0];
    request.depthRequested = true;
    request.depthResolution = depthProfile.resolution;
    request.depthFramerate = depthProfile.framerates[0];
    request.pointCloudRequested = pointcloud_stream;

    std::unique_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    requestedRGBDStreams_[stream_id] = request;
    lock.unlock();
    updateProfile();
    sleepGrabber_ = false;
    startFrameGrabber();
    return true;
}

void RGBDCameraManager::removeFrameStream(int stream_id) {
    logger_->debug("stopFrameStream()");
    std::unique_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    requestedRGBDStreams_.erase(stream_id);

    if (requestedRGBDStreams_.size() == 0) {
        logger_->info("stopFrameStream(): No more streams active, sleeping grabber");
        sleepGrabber_ = true;
        depthRequested_ = false;
        pointCloudRequested_ = false;
        return;
    }
    lock.unlock();
    updateProfile();
}

void RGBDCameraManager::frameGrabber() {
    logger_->debug("frameGrabber()");
    RGBDPointCloud pointcloud;
    cv::rgbd::RgbdFrame frame;
    while (!stopGrabber_) {
        if (sleepGrabber_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if (!initializeDevice()) {
            logger_->error("frameGrabber(): Cannot initialize the camera");
            break;
        }

        if (pointCloudRequested_) {
            std::unique_lock<std::mutex> lockCam(cameraMtx_);
            pointcloud  = rgbdCamera_->capturePointCloud();
            lockCam.unlock();
            if (pointcloud.frame.image.empty() || pointcloud.frame.depth.empty()) continue;
            std::unique_lock<std::shared_timed_mutex> lockFrm(frameMtx_);
            latestPointCloud_ = pointcloud;
            lockFrm.unlock();
            continue;
        }

        {
            std::unique_lock<std::mutex> lock(cameraMtx_);
            frame = rgbdCamera_->captureImageAndDepth();
            lock.unlock();
        }
        if (frame.image.empty() || frame.depth.empty()) {
            logger_->warn("frameGrabber(): Empty frame");
            continue;
        }

        /**
         * Even thought the cameraMtx_ should be explicitly unlocked,
         * the compiler optimizes the code to the point where the mutex barely gets
         * unlocked. This creates timeouts and long waits to access the camera
         * Adding a small delay of 1 ms ensures that the mutex gets unlocked, there
         * are probably better ways of doing this.
         * (jplayang)
         *
         */
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        std::unique_lock<std::shared_timed_mutex> lock(frameMtx_);
        latestPointCloud_.frame = frame;
        latestPointCloud_.pointcloud.reset();
        lock.unlock();
    }
    logger_->debug("frameGrabber(): Exiting frame grabber");
}

bool RGBDCameraManager::updateProfile() {
    logger_->debug("updateProfile()");
    cv::Size reqResolution = getMaxRequestedResolution();
    uint64_t reqFramerate = getMaxRequestedFramerate();
    depthRequested_ = isDepthRequested();
    pointCloudRequested_ = isPointCloudRequested();

    if (currentProfile_.resolution != reqResolution || currentProfile_.framerates[0] != reqFramerate) {  // NOLINT
        auto it = std::find_if(cameraProfiles_.begin(), cameraProfiles_.end(),
            [reqResolution](Profile p){ return p.resolution == reqResolution;});
        if (std::find(it->framerates.begin(), it->framerates.end(), reqFramerate) == it->framerates.end())  // NOLINT
            reqFramerate = *std::max_element(it->framerates.begin(), it->framerates.end());
    }

    if (!depthRequested_) {
        std::unique_lock<std::mutex> lock(cameraMtx_);
        if (!rgbdCamera_->setProfile(Profile(reqResolution, reqFramerate))) {
            logger_->warn("updateProfile(): Set profile error");
            return false;
        }
        lock.unlock();
    } else {
        cv::Size reqDepthResolution = getMaxRequestedDepthResolution();
        uint64_t reqDepthFramerate = getMaxRequestedDepthFramerate();

        if (currentDepthProfile_.resolution != reqDepthResolution ||
            currentDepthProfile_.framerates[0] != reqDepthFramerate) {
            auto it = std::find_if(depthProfiles_.begin(), depthProfiles_.end(),
                [reqDepthResolution](Profile p){
                    return p.resolution == reqDepthResolution;
                });
            if (std::find(it->framerates.begin(), it->framerates.end(), reqDepthFramerate) == it->framerates.end())  // NOLINT
                reqDepthFramerate = *std::max_element(it->framerates.begin(), it->framerates.end());
        }

        std::unique_lock<std::mutex> lock(cameraMtx_);
        if (!rgbdCamera_->setProfile(
            Profile(reqResolution, reqFramerate),
            Profile(reqDepthResolution, reqDepthFramerate))) {
            logger_->warn("updateProfile(): Set depth profile error");
            return false;
        }
        lock.unlock();

        currentDepthProfile_.resolution = reqDepthResolution;
        currentDepthProfile_.framerates[0] = reqDepthFramerate;
    }
    currentProfile_.resolution = reqResolution;
    currentProfile_.framerates[0] = reqFramerate;
    return true;
}

bool RGBDCameraManager::isDepthRequested() {
    std::shared_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    for (auto it = requestedRGBDStreams_.begin(); it != requestedRGBDStreams_.end(); ++it) {
        if (it->second.depthRequested) return true;
    }
    return false;
}

bool RGBDCameraManager::isPointCloudRequested() {
    std::shared_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    for (auto it = requestedRGBDStreams_.begin(); it != requestedRGBDStreams_.end(); ++it) {
        if (it->second.pointCloudRequested) return true;
    }
    return false;
}

cv::Size RGBDCameraManager::getMaxRequestedDepthResolution() {
    cv::Size max_resolution_requested(0, 0);
    std::shared_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    for (auto it = requestedRGBDStreams_.begin(); it != requestedRGBDStreams_.end(); ++it) {
        cv::Size resolution = it->second.depthResolution;
        if ((resolution.width * resolution.height) >
            (max_resolution_requested.width * max_resolution_requested.height)) {
                max_resolution_requested = resolution;
        }
    }
    return max_resolution_requested;
}

int RGBDCameraManager::getMaxRequestedDepthFramerate() {
    int max_framerate = 0;
    std::shared_lock<std::shared_timed_mutex> lock(requestedStreamsMtx_);
    for (auto it = requestedRGBDStreams_.begin(); it != requestedRGBDStreams_.end(); ++it) {
        int framerate = it->second.depthFramerate;
        if (framerate > max_framerate)
            max_framerate = framerate;
    }
    return max_framerate;
}

}   // namespace rgbdcameras
}   // namespace sensors
}   // namespace crf
