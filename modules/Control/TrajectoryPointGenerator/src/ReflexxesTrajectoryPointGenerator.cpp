/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <vector>
#include <cmath>

#include "TrajectoryPointGenerator/ReflexxesTrajectoryPointGenerator.hpp"

#define DEFAULT_JERK_MULTIPL 5
#define TASK_SPACE_SIZE 6

namespace crf::control::trajectorypointgenerator {

ReflexxesTrajectoryPointGenerator::ReflexxesTrajectoryPointGenerator(ControlMode controlMode,
    std::vector<bool> dimSelection, float cycleTimeInS):
    logger_("ReflexxesTrajectoryPointGenerator"),
    controlMode_(controlMode),
    numberOfDOFs_(dimSelection.size()),
    generator_(numberOfDOFs_, cycleTimeInS),
    resultValue_(),
    dimVector_(dimSelection),
    status_(ControlStatus::NOT_INITIALIZED),
    trajPoint_{},
    trajPointMutex_(),
    posOutputParam_(numberOfDOFs_),
    velOutputParam_(numberOfDOFs_),
    posInputParam_(numberOfDOFs_),
    velInputParam_(numberOfDOFs_),
    velFlags_(),
    posFlags_() {
    logger_->debug("CTor");
    if (numberOfDOFs_ != TASK_SPACE_SIZE) {
        throw std::runtime_error("Provided dimensions are invalid");
    }
    RMLBoolVector selectionVector(numberOfDOFs_);
    selectionVector.Set(false);
    int countSelected = 0;
    for (uint dimID = 0; dimID < numberOfDOFs_; dimID++) {
        countSelected += dimVector_[dimID];
        selectionVector[dimID] = dimVector_[dimID];
    }

    if (!countSelected) {
        throw std::runtime_error("No dimension for generating trajectories selected");
    }
    switch (controlMode_) {
        case ControlMode::VELOCITY: {
            velInputParam_.SetSelectionVector(selectionVector);
            velFlags_.SynchronizationBehavior = RMLFlags::NO_SYNCHRONIZATION;
            setNewTrajectoryPoint(velInputParam_);
            break;
        }
        case ControlMode::POSITION: {
            posInputParam_.SetSelectionVector(selectionVector);
            setNewTrajectoryPoint(posInputParam_);
            break;
        }
        default: {
            status_ = ControlStatus::ERROR;
            throw std::runtime_error("The selected control mode is invalid");
        }
    }
    logger_->info("Created Reflexxes Trajectory Point Controller");
}

ReflexxesTrajectoryPointGenerator::~ReflexxesTrajectoryPointGenerator() {
    logger_->debug("DTor");
    status_ = ControlStatus::REACHED_FINAL_STATE;
}

ControlMode ReflexxesTrajectoryPointGenerator::getControlMode() const {
    logger_->debug("getControlMode");
    return controlMode_;
}

boost::optional<utility::types::TaskTrajectoryData> ReflexxesTrajectoryPointGenerator::
    getTaskTrajectoryPoint() const {
    logger_->debug("getTaskTrajectoryPoint");
    if (status_ == ControlStatus::ERROR || status_ == ControlStatus::NOT_INITIALIZED) {
        return boost::none;
    }
    std::lock_guard<std::mutex> lg(trajPointMutex_);
    return trajPoint_;
}

bool ReflexxesTrajectoryPointGenerator::updatePositionTarget(
        const utility::types::TaskPose& targetPosition) {
    logger_->debug("updatePositionTarget");
    if (status_ == ControlStatus::ERROR) {
        logger_->debug("updatePositionTarget not possible due to error state");
        return false;
    }
    if (controlMode_ != ControlMode::POSITION) {
        logger_->debug("updatePositionTarget not possible in Velocity Mode");
        return false;
    }
    size_t currentInputDimension = 0;
    for (uint i = 0; i < 3; i++) {
        if (dimVector_[i]) {
            posInputParam_.TargetPositionVector->VecData[currentInputDimension] =
                targetPosition.getPosition()(i);
            currentInputDimension++;
        }
    }
    for (uint i = 3; i < 6; i++) {
        if (dimVector_[i]) {
            posInputParam_.TargetPositionVector->VecData[currentInputDimension] =
                targetPosition.getCardanXYZ()[i - 3];
            currentInputDimension++;
        }
    }
    if (!checkValidity(posInputParam_)) {
        logger_->error("input values are still invalid - may specify max vel and pos target");
    }
    return true;
}

bool ReflexxesTrajectoryPointGenerator::updateVelocityTarget(
        const utility::types::TaskVelocity& targetVelocity) {
    logger_->debug("updateVelocityTarget");
    if (status_ == ControlStatus::ERROR) {
        logger_->debug("updateVelocityTarget not possible due to error state");
        return false;
    }
    if (controlMode_ == ControlMode::VELOCITY) {
        size_t currentInputDimension = 0;
        for (uint i = 0; i < numberOfDOFs_; i++) {
            if (dimVector_[i]) {
                velInputParam_.TargetVelocityVector->VecData[currentInputDimension] =
                    targetVelocity[i];
                currentInputDimension++;
            }
        }
       return checkValidity(velInputParam_);
    } else if (controlMode_ == ControlMode::POSITION) {
        size_t currentInputDimension = 0;
        for (uint i = 0; i < numberOfDOFs_; i++) {
            if (dimVector_[i]) {
                posInputParam_.TargetVelocityVector->VecData[currentInputDimension] =
                    targetVelocity[i];
                currentInputDimension++;
            }
        }
        if (!checkValidity(posInputParam_)) {
            logger_->error("input values are still invalid - may specify max vel and pos target");
        }
       return true;
    }
    return false;
}

bool ReflexxesTrajectoryPointGenerator::updateCurrentState(
        const utility::types::TaskTrajectoryData& currentState) {
    if (status_ == ControlStatus::ERROR || status_ == REACHED_FINAL_STATE) {
        return false;
    }
    if (!setCurrentState(currentState)) {
        logger_->debug("something went wrong in setCurrentState");
        return false;
    }

    if (status_ == ControlStatus::NOT_INITIALIZED) {
        if (controlMode_ == ControlMode::VELOCITY) {
            setNewTrajectoryPoint(velInputParam_);
        } else if (controlMode_ == ControlMode::POSITION) {
            setNewTrajectoryPoint(posInputParam_);
        } else {
            return false;
        }
    } else {
        return computeNewState();
    }
    return true;
}

bool ReflexxesTrajectoryPointGenerator::updateMotionConstraints(
        const utility::types::TaskTrajectoryData& maximumState) {
    logger_->debug("updateMotionConstraints");
    if (status_ == ControlStatus::ERROR) {
        logger_->debug("updateMotionConstraints not possible due to error state");
        return false;
    }

    if (controlMode_ == ControlMode::VELOCITY) {
        for (uint dimID = 0; dimID < numberOfDOFs_; dimID++) {
            velInputParam_.MaxAccelerationVector->VecData[dimID] =
                maximumState.acceleration[dimID];
            velInputParam_.MaxJerkVector->VecData[dimID] =
                DEFAULT_JERK_MULTIPL*maximumState.acceleration[dimID];
        }
        return checkValidity(velInputParam_);
    } else if (controlMode_ == ControlMode::POSITION) {
        for (uint dimID = 0; dimID < numberOfDOFs_; dimID++) {
            posInputParam_.MaxVelocityVector->VecData[dimID] =
                maximumState.velocity[dimID];
            posInputParam_.MaxAccelerationVector->VecData[dimID] =
                maximumState.acceleration[dimID];
            posInputParam_.MaxJerkVector->VecData[dimID] =
                DEFAULT_JERK_MULTIPL*maximumState.acceleration[dimID];
        }
        if (!checkValidity(posInputParam_)) {
            logger_->error("input values are still invalid - may specify max vel and pos target");
        }
        return true;
    }
    return false;
}

bool ReflexxesTrajectoryPointGenerator::computeNewState() {
    logger_->debug("computeNewState");
    if (status_ == ControlStatus::NOT_INITIALIZED) {
        logger_->debug("computeNewState not possible due to not-initialized state");
        return false;
    }
    utility::types::TaskTrajectoryData newTrajPoint;
    resultValue_ = ReflexxesAPI::RML_ERROR;

    if (controlMode_ == ControlMode::VELOCITY) {
        resultValue_ = (ReflexxesAPI::RMLResultValue)
            generator_.RMLVelocity(velInputParam_, &velOutputParam_, velFlags_);
        newTrajPoint = getNewTrajectoryPoint(velOutputParam_);
    } else if (controlMode_ == ControlMode::POSITION) {
        resultValue_ = (ReflexxesAPI::RMLResultValue)
            generator_.RMLPosition(posInputParam_, &posOutputParam_, posFlags_);
        newTrajPoint = getNewTrajectoryPoint(posOutputParam_);
    } else {
        return false;
    }
    if (resultValue_ < 0) {
        logger_->error("An error occurred {}.", resultValue_);
        status_ = ControlStatus::ERROR;
        return false;
    }
    if (resultValue_ == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
        status_ = ControlStatus::REACHED_FINAL_STATE;
    }
    std::lock_guard<std::mutex> lg(trajPointMutex_);
    trajPoint_ = newTrajPoint;
    return true;
}

bool ReflexxesTrajectoryPointGenerator::setCurrentState(
    const utility::types::TaskTrajectoryData& currentState) {
    logger_->debug("setCurrentState");
    if (controlMode_ == ControlMode::VELOCITY) {
        size_t currentInputDimension = 0;
        for (size_t i = 0; i < 3; i++) {
            if (dimVector_[i]) {
                velInputParam_.CurrentPositionVector->VecData[currentInputDimension] =
                    currentState.pose.getPosition()(i);
                currentInputDimension++;
            }
        }
        for (size_t i = 3; i < 6; i++) {
            if (dimVector_[i]) {
                velInputParam_.CurrentPositionVector->VecData[currentInputDimension] =
                    currentState.pose.getCardanXYZ()[i - 3];
                currentInputDimension++;
            }
        }
        currentInputDimension = 0;
        for (uint i = 0; i < numberOfDOFs_; i++) {
            if (dimVector_[i]) {
                velInputParam_.CurrentVelocityVector->VecData[currentInputDimension] =
                    currentState.velocity[i];
                velInputParam_.CurrentAccelerationVector->VecData[currentInputDimension] =
                    currentState.acceleration[i];
                currentInputDimension++;
            }
        }
    } else if (controlMode_ == ControlMode::POSITION) {
        size_t currentInputDimension = 0;
        for (size_t i = 0; i < 3; i++) {
            if (dimVector_[i]) {
                posInputParam_.CurrentPositionVector->VecData[currentInputDimension] =
                    currentState.pose.getPosition()(i);
                currentInputDimension++;
            }
        }
        for (size_t i = 3; i < 6; i++) {
            if (dimVector_[i]) {
                posInputParam_.CurrentPositionVector->VecData[currentInputDimension] =
                    currentState.pose.getCardanXYZ()[i - 3];
                currentInputDimension++;
            }
        }

        currentInputDimension = 0;
        for (uint i = 0; i < numberOfDOFs_; i++) {
            if (dimVector_[i]) {
                posInputParam_.CurrentVelocityVector->VecData[currentInputDimension] =
                    currentState.velocity[i];
                posInputParam_.CurrentAccelerationVector->VecData[currentInputDimension] =
                    currentState.acceleration[i];
                currentInputDimension++;
            }
        }
    } else {
        return false;
    }
    return true;
}

bool ReflexxesTrajectoryPointGenerator::checkValidity(const
        RMLInputParameters& inputParam) {
    logger_->debug("checkValidity");
    if (!inputParam.CheckForValidity()) {
        logger_->debug("Input values are invalid");
        status_ = ControlStatus::NOT_INITIALIZED;
        return false;
    }
    status_ = ControlStatus::OKAY;
    return true;
}

bool ReflexxesTrajectoryPointGenerator::setNewTrajectoryPoint(const
        RMLInputParameters& inputParam) {
    logger_->debug("setNewTrajectoryPoint");
    if (status_ == ControlStatus::ERROR) {
        logger_->debug("setNewTrajectoryPoint not possible due to error state");
        return false;
    }

    std::array<double, 6> posVector;
    size_t currentInputDimension = 0;
    for (uint i = 0; i < numberOfDOFs_; i++) {
        if (dimVector_[i]) {
            posVector[i] = inputParam.CurrentPositionVector->VecData[currentInputDimension];
            trajPoint_.velocity[i] =
                inputParam.CurrentVelocityVector->VecData[currentInputDimension];
            trajPoint_.acceleration[i] =
                inputParam.CurrentAccelerationVector->VecData[currentInputDimension];
            currentInputDimension++;
        } else {
            posVector[i] = std::numeric_limits<double>::quiet_NaN();
            trajPoint_.velocity[i] = std::numeric_limits<double>::quiet_NaN();
            trajPoint_.acceleration[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    trajPoint_.pose = crf::utility::types::TaskPose({posVector[0], posVector[1], posVector[2]},
        crf::math::rotation::CardanXYZ({posVector[3], posVector[4], posVector[5]}));
    return true;
}

utility::types::TaskTrajectoryData ReflexxesTrajectoryPointGenerator::
    getNewTrajectoryPoint(const RMLOutputParameters& outputParam) {
    logger_->debug("getNewTrajectoryPoint");
    std::array<double, 6> posVector;
    utility::types::TaskTrajectoryData localTrajPoint;
    size_t currentOutputDimension = 0;
    for (uint i = 0; i < numberOfDOFs_; i++) {
        if (dimVector_[i]) {
            posVector[i] = outputParam.NewPositionVector->VecData[currentOutputDimension];
            localTrajPoint.velocity[i] =
                outputParam.NewVelocityVector->VecData[currentOutputDimension];
            localTrajPoint.acceleration[i] =
                outputParam.NewAccelerationVector->VecData[currentOutputDimension];
            currentOutputDimension++;
        } else {
            posVector[i] = std::numeric_limits<double>::quiet_NaN();
            localTrajPoint.velocity[i] = std::numeric_limits<double>::quiet_NaN();
            localTrajPoint.acceleration[i] = std::numeric_limits<double>::quiet_NaN();
        }
    }

    localTrajPoint.pose = crf::utility::types::TaskPose({posVector[0], posVector[1], posVector[2]},
        crf::math::rotation::CardanXYZ({posVector[3], posVector[4], posVector[5]}));
    return localTrajPoint;
}


}  // namespace crf::control::trajectorypointgenerator
