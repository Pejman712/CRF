/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <algorithm>

#include "RobotPoseEstimators/RobotPoseEstimator.hpp"
#include "RobotPoseEstimators/RobotPoseEstimatorTrackingCam.hpp"

#define ACCELEROMETER_ACQUISITION_US 20000

namespace crf {
namespace applications {
namespace robotposeestimator {

RobotPoseEstimatorTrackingCam::RobotPoseEstimatorTrackingCam(
    std::shared_ptr<Kalman::UnscentedKalmanFilter<SystemState>> ukf,
    std::shared_ptr<robots::robotbase::IRobotBase> base,
    std::shared_ptr<sensors::trackingcamera::ITrackingCamera> camera):
    logger_("RobotPoseEstimatorTrackingCam"),
    camera_(camera),
    cameraThread_(),
    estimator_(new RobotPoseEstimator(ukf, base)),
    stopAll_(true) {
    logger_->debug("CTor");
    if (!camera) {
        throw std::runtime_error("No tracking camera provided");
    }
}

RobotPoseEstimatorTrackingCam::~RobotPoseEstimatorTrackingCam() {
    deinitialize();
    logger_->debug("DTor");
}

bool RobotPoseEstimatorTrackingCam::initialize() {
    logger_->info("initialize");
    if (!estimator_->initialize()) {
        return false;
    }
    if (!camera_->initialize()) {
        logger_->debug("Tracking Camera could not be initialized");
        return false;
    }
    if (!camera_->resetPose()) {
        logger_->debug("Tracking Camera pose could not be reset");
        return false;
    }
    stopAll_ = false;
    cameraThread_ = std::thread([this]() {
        while (!stopAll_) {
            dataAcquisitionLoopCamera();
        }
    });
    return true;
}

bool RobotPoseEstimatorTrackingCam::deinitialize() {
    logger_->debug("deinitialize");
    if (!estimator_->deinitialize()) {
        return false;
    }
    stopAll_ = true;
    logger_->debug("Waiting for robot base thread to join");
    if (cameraThread_.joinable()) {
        cameraThread_.join();
    }
    if (!camera_->deinitialize()) {
        logger_->debug("Tracking Camera could not be deinitialized");
        return false;
    }
    return true;
}

boost::optional<utility::types::TaskPose> RobotPoseEstimatorTrackingCam::getPosition() {
    logger_->debug("getPose");
    if (stopAll_) {
        logger_->error("Thread not running - Returning empty vector");
        return boost::none;
    }
    auto dataFromCamValid = obtainTrackingData();
    if (!dataFromCamValid) {
        return boost::none;
    }
    auto data = dataFromCamValid.get();
    // Coordinate system changed according to Intel T265 sensor specifications
    utility::types::TaskPose result({-data.pose.translation.z,
        data.pose.translation.x, data.pose.translation.y, -data.gyro.z, data.gyro.x, data.gyro.y});
    return result;
}

boost::optional<utility::types::TaskVelocity> RobotPoseEstimatorTrackingCam::getVelocity() {
    logger_->debug("getVelocity");
    if (stopAll_) {
        logger_->error("Thread not running - Returning empty vector");
        return boost::none;
    }
    auto dataFromCamValid = obtainTrackingData();
    if (!dataFromCamValid) {
        return boost::none;
    }
    auto data = dataFromCamValid.get();
    // Coordinate system changed according to Intel T265 sensor specifications
    utility::types::TaskVelocity result({-data.pose.velocity.z,
        data.pose.velocity.x, data.pose.velocity.y, -data.pose.angular_velocity.z,
        data.pose.angular_velocity.x, data.pose.angular_velocity.y});
    return result;
}

boost::optional<utility::types::TaskAcceleration> RobotPoseEstimatorTrackingCam::
    getAcceleration() {
    logger_->debug("getAcceleration");
    if (stopAll_) {
        logger_->error("Thread not running - Returning empty vector");
        return boost::none;
    }
    auto dataFromCamValid = obtainTrackingData();
    if (!dataFromCamValid) {
        return boost::none;
    }
    auto data = dataFromCamValid.get();
    // Coordinate system changed according to Intel T265 sensor specifications
    utility::types::TaskAcceleration result({-data.pose.acceleration.z,
        data.pose.acceleration.x, data.pose.acceleration.y, -data.pose.angular_acceleration.z,
        data.pose.angular_acceleration.x, data.pose.angular_acceleration.y});
    return result;
}

boost::optional<sensors::trackingcamera::TrackingData> RobotPoseEstimatorTrackingCam::
    obtainTrackingData() {
    logger_->debug("obtainTrackingData");
    sensors::trackingcamera::TrackingData data = camera_->getTrackingData();
    if ((data.pose.tracker_confidence == 0x0) || (data.pose.mapper_confidence == 0x0)) {
        logger_->error("Data unreliable - Returning empty vector");
        return boost::none;
    }
    return data;
}

bool RobotPoseEstimatorTrackingCam::feedVelocityData() {
    auto estimatedStateValid = estimator_->getVelocity();
    if (!estimatedStateValid) {
        logger_->error("Estimated state invalid");
        return false;
    }
    auto estimatedVelocity = estimatedStateValid.get();

    logger_->debug("fetchedEstimate from TrackingCam");

    // Coordinate system in cam converted to robot base coordinate system
    // z in camera equals x in robot, x in camera equals y in robot, y in camera equals z in robot
    std::array<float, 3> velocities{
        estimatedVelocity(1), estimatedVelocity(5), -estimatedVelocity(0)};
    if (!camera_->feedBaseVelocity(velocities)) {
        logger_->error("Could not feed velocity to camera");
        return false;
    }
    return true;
}

void RobotPoseEstimatorTrackingCam::dataAcquisitionLoopCamera() {
    auto start = std::chrono::high_resolution_clock::now();
    bool dataToCamValid = feedVelocityData();
    if (!dataToCamValid) {
        stopAll_ = true;
    }
    std::chrono::microseconds elapsed
            = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - start);
    if ((std::chrono::microseconds(ACCELEROMETER_ACQUISITION_US) - elapsed).count() > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(
            ACCELEROMETER_ACQUISITION_US) - elapsed);
    }
}

}  // namespace robotposeestimator
}  // namespace applications
}  // namespace crf
