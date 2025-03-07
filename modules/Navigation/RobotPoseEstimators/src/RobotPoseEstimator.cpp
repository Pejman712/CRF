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
#include <exception>
#include <Eigen/Dense>

#include "RobotPoseEstimators/RobotPoseEstimator.hpp"

#define IMU_AQUISITION_INTERVAL_US 5000

namespace crf {
namespace applications {
namespace robotposeestimator {

RobotPoseEstimator::RobotPoseEstimator(
    std::shared_ptr<Kalman::UnscentedKalmanFilter<SystemState>> ukf,
    std::shared_ptr<robots::robotbase::IRobotBase> base,
    std::shared_ptr<sensors::imu::IIMU> imu):
    logger_("RobotPoseEstimator"),
    base_(base),
    imu_(imu),
    baseConfiguration_(base_->getConfiguration()),
    baseDataAcquisitonLoopTime_(std::chrono::microseconds(
                baseConfiguration_->getRTLoopTime())),
    stopAll_(true),
    baseThread_(),
    imuThread_(),
    estimate_{},
    ukf_(ukf),
    encodersModel_(baseConfiguration_) {
    logger_->debug("CTor");
    x_.setZero();
    ukf_->init(x_);
}

RobotPoseEstimator::~RobotPoseEstimator() {
    deinitialize();
    logger_->debug("DTor");
}

bool RobotPoseEstimator::initialize() {
    logger_->info("initialize");
    if (baseThread_.joinable()) {
        logger_->debug("Threads are running already");
        return false;
    }
    baseThread_ = std::thread([this]() {
        while (!stopAll_) {
            dataAcquisitionLoopRobot();
        }
    });
    if (imu_) {
        if (!imu_->initialize()) {
            logger_->warn("Could not initialize IMU");
            return false;
        }
        imuThread_ = std::thread([this]() {
            logger_->debug("Initalizing data acquisition on imu");
            while (!stopAll_) {
                dataAcquisitionLoopIMU();
            }
        });
    }
    stopAll_ = false;
    return true;
}

bool RobotPoseEstimator::deinitialize() {
    logger_->debug("deinitialize");
    if (!baseThread_.joinable()) {
        logger_->warn("Thread is not joinable");
        return false;
    }
    stopAll_ = true;

    logger_->debug("Waiting for robot base thread to join");
    baseThread_.join();
    if (imuThread_.joinable()) {
        imuThread_.join();
    }
    return true;
}


boost::optional<utility::types::TaskPose> RobotPoseEstimator::getPosition() {
    logger_->debug("getPosition");
    if (!fetchEstimate()) {
        logger_->error("No initialization - Returning empty vector");
        return boost::none;
    }
    return estimate_.position;
}

boost::optional<utility::types::TaskVelocity> RobotPoseEstimator::getVelocity() {
    logger_->debug("getVelocity");
    if (!fetchEstimate()) {
        logger_->warn("No initialization - Returning empty vector");
        return boost::none;
    }
    return estimate_.velocity;
}

boost::optional<utility::types::TaskAcceleration> RobotPoseEstimator::getAcceleration() {
    logger_->debug("getAcceleration");
    if (!fetchEstimate()) {
        logger_->warn("No initialization - Returning empty vector");
        return boost::none;
    }
    return estimate_.acceleration;
}

bool RobotPoseEstimator::fetchEstimate() {
    // logger_->debug("fetchEstimate");
    if (stopAll_) {
        logger_->debug("cannot fetch estimate if threads are stopped");
        return false;
    }
    x_ = ukf_->predict(systemModel_);
    estimate_.velocity = {x_(0), x_(1), .0, .0, .0, x_(6)};
    estimate_.acceleration = {x_(3), x_(4), x_(5), .0, .0, .0};
    // logger_->debug("fetchedEstimate");
    return true;
}

void RobotPoseEstimator::dataAcquisitionLoopRobot() {
    auto start = std::chrono::high_resolution_clock::now();
    auto motorVelocitiesValid = base_->getMotorsVelocities();
    if (!motorVelocitiesValid) {
        logger_->error("feedback from robot invalid");
        stopAll_ = true;
    } else {
        auto vectorMotorVelocities = motorVelocitiesValid.get();
        if (vectorMotorVelocities.size() != baseConfiguration_->getRobotParameters().wheelsCount) {
            logger_->error("number of motors received invalid");
            stopAll_ = true;
        } else {
        VelocityMeasurement encoders;
        encoders.frontLeft() = vectorMotorVelocities[0];
        encoders.frontRight() = vectorMotorVelocities[1];
        encoders.backLeft() = vectorMotorVelocities[2];
        encoders.backRight() = vectorMotorVelocities[3];
        ukf_->predict(systemModel_);
        x_ = ukf_->update(encodersModel_, encoders);
        if (!updatePosition()) {
            logger_->error("could not update estimated position");
            stopAll_ = true;
        }
        }
    }
    std::chrono::microseconds elapsed
            = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - start);
    if ((baseDataAcquisitonLoopTime_ - elapsed).count() > 0) {
        std::this_thread::sleep_for(baseDataAcquisitonLoopTime_ - elapsed);
    } else {
        logger_->error("exection time longer than real time loop");
    }
}

void RobotPoseEstimator::dataAcquisitionLoopIMU() {
    auto start = std::chrono::high_resolution_clock::now();

    auto measuredAccelerations = imu_->getIMUData().acceleration;
    if (measuredAccelerations.size() != 3) {
        logger_->error("imu data invalid, size: {}", measuredAccelerations.size());
        stopAll_ = true;
    }
    AccelerationMeasurement imus;
    imus.accelerationX() = measuredAccelerations[0];
    imus.accelerationY() = measuredAccelerations[1];
    imus.accelerationZ() = measuredAccelerations[2];
    ukf_->predict(systemModel_);
    ukf_->update(imuModel_, imus);
    std::chrono::microseconds elapsed
            = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::high_resolution_clock::now() - start);
    if ((std::chrono::microseconds(IMU_AQUISITION_INTERVAL_US) - elapsed).count() > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(
            IMU_AQUISITION_INTERVAL_US) - elapsed);
    }
}

bool RobotPoseEstimator::updatePosition() {
    Eigen::Vector3f currentVelocities;
    auto validEstimate = fetchEstimate();
    if (!validEstimate) {
        return false;
    }
    auto estimatedPosition = estimate_.position;
    currentVelocities(0) = estimate_.velocity(0);
    currentVelocities(1) = estimate_.velocity(1);
    currentVelocities(2) = estimate_.velocity(5);
    auto rotationMatrixArray = estimatedPosition.getPosRotMatrix();
    Eigen::Matrix3f rotationMatrix;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotationMatrix(i, j) = rotationMatrixArray[j+i*3+3];
        }
    }
    Eigen::Vector3f rotatedVelocities = rotationMatrix*currentVelocities;
    utility::types::TaskVelocity rotatedTaskVelocities = {rotatedVelocities(0),
        rotatedVelocities(1), .0, .0, .0, rotatedVelocities(2)};
    auto travelledDistance = rotatedTaskVelocities*baseDataAcquisitonLoopTime_.count()*(1e-6);
    utility::types::TaskPose updatedPosition({
        estimate_.position.getPosition()[0] + travelledDistance(0),
        estimate_.position.getPosition()[1] + travelledDistance(1), .0, .0, .0,
        estimate_.position.getCardanXYZ()[2] + travelledDistance(5)});
    estimate_.position = updatedPosition;
    return true;
}

}  // namespace robotposeestimator
}  // namespace applications
}  // namespace crf
