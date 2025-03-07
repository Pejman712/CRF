#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "RobotBase/IRobotBase.hpp"
#include "IMU/IIMU.hpp"
#include "RobotPoseEstimators/IRobotPoseEstimator.hpp"
#include "RobotPoseEstimators/RobotPoseEstimationMeasurementModels.hpp"
#include "RobotPoseEstimators/RobotPoseEstimationSystemModel.hpp"

#include "StateEstimator/kalman/UnscentedKalmanFilter.hpp"

namespace crf {
namespace applications {
namespace robotposeestimator {

class RobotPoseEstimator : public IRobotPoseEstimator {
 public:
    RobotPoseEstimator() = delete;
    RobotPoseEstimator(std::shared_ptr<Kalman::UnscentedKalmanFilter<SystemState>> ukf,
                std::shared_ptr<robots::robotbase::IRobotBase> base,
                std::shared_ptr<sensors::imu::IIMU> imu = nullptr);
    RobotPoseEstimator(const RobotPoseEstimator& other) = delete;
    RobotPoseEstimator(RobotPoseEstimator&& other) = delete;
    ~RobotPoseEstimator() override;
    bool initialize() override;
    bool deinitialize() override;
    boost::optional<utility::types::TaskPose> getPosition() override;
    boost::optional<utility::types::TaskVelocity> getVelocity() override;
    boost::optional<utility::types::TaskAcceleration> getAcceleration() override;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<robots::robotbase::IRobotBase> base_;
    std::shared_ptr<sensors::imu::IIMU> imu_;
    std::shared_ptr<robots::robotbase::RobotBaseConfiguration> baseConfiguration_;
    const std::chrono::microseconds baseDataAcquisitonLoopTime_;
    std::atomic<bool> stopAll_;
    std::thread baseThread_;
    std::thread imuThread_;

    utility::types::TaskTrajectoryData estimate_;

    std::shared_ptr<Kalman::UnscentedKalmanFilter<SystemState>> ukf_;
    SystemState x_;
    SystemModel systemModel_;
    VelocityMeasurementModel encodersModel_;
    AccelerationMeasurementModel imuModel_;
    // Using macro conditionnally (depending on template parameters)
    // see more in https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    enum {
        NeedsToAlign = (sizeof(x_)%16) == 0
    };
    bool fetchEstimate();
    void dataAcquisitionLoopRobot();
    void dataAcquisitionLoopIMU();
    bool updatePosition();
};

}  // namespace robotposeestimator
}  // namespace applications
}  // namespace crf
