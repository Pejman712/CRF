#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <fstream>

#include "EventLogger/EventLogger.hpp"
#include "RobotBase/IRobotBase.hpp"
#include "TrackingCamera/ITrackingCamera.hpp"

#include "RobotPoseEstimators/RobotPoseEstimator.hpp"

namespace crf {
namespace applications {
namespace robotposeestimator {

/*
 * This class implements an estimator for position, velocity and TaskAcceleration
 * of a mobile robotic platform, hereby we pass the robots'velocities to a tracking camera
 * which integrates a VSLAM algorithm to compute the current pose estimation
 */

class RobotPoseEstimatorTrackingCam : public IRobotPoseEstimator {
 public:
    RobotPoseEstimatorTrackingCam() = delete;
    RobotPoseEstimatorTrackingCam(
        std::shared_ptr<Kalman::UnscentedKalmanFilter<SystemState>> ukf,
        std::shared_ptr<robots::robotbase::IRobotBase> base,
        std::shared_ptr<sensors::trackingcamera::ITrackingCamera> camera);
    ~RobotPoseEstimatorTrackingCam() override;
    bool initialize() override;
    bool deinitialize() override;
    boost::optional<utility::types::TaskPose> getPosition() override;
    boost::optional<utility::types::TaskVelocity> getVelocity() override;
    boost::optional<utility::types::TaskAcceleration> getAcceleration() override;
 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<sensors::trackingcamera::ITrackingCamera> camera_;
    std::thread cameraThread_;
    std::shared_ptr<RobotPoseEstimator> estimator_;
    std::atomic<bool> stopAll_;
    void dataAcquisitionLoopCamera();
    bool feedVelocityData();
    boost::optional<sensors::trackingcamera::TrackingData> obtainTrackingData();
};

}  // namespace robotposeestimator
}  // namespace applications
}  // namespace crf
