/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <vector>

#include <ReflexxesAPI.h>

#include "EventLogger/EventLogger.hpp"
#include "TrajectoryPointGenerator/ITrajectoryPointGenerator.hpp"

namespace crf::control::trajectorypointgenerator {

class ReflexxesTrajectoryPointGenerator : public ITrajectoryPointGenerator {
 public:
    ReflexxesTrajectoryPointGenerator() = delete;
    ReflexxesTrajectoryPointGenerator(ControlMode controlMode,
            std::vector<bool> dimSelection, float cycleTimeInS);
    ReflexxesTrajectoryPointGenerator(const ReflexxesTrajectoryPointGenerator&) = default;
    ~ReflexxesTrajectoryPointGenerator() override;

    ControlMode getControlMode() const override;
    boost::optional<utility::types::TaskTrajectoryData>
        getTaskTrajectoryPoint() const override;
    bool updatePositionTarget(
        const utility::types::TaskPose& targetPosition) override;
    bool updateVelocityTarget(
        const utility::types::TaskVelocity& targetVelocity) override;
    bool updateCurrentState(
        const utility::types::TaskTrajectoryData& currentState) override;
    bool updateMotionConstraints(
        const utility::types::TaskTrajectoryData& maximumState) override;

    enum ControlStatus {
        OKAY,                 //!< Everything is okay
        REACHED_FINAL_STATE,  //!< Final State is reached.
        ERROR,                //!< Some error occured.
        NOT_INITIALIZED       //!< Input parameters are invalid
    };

 private:
    utility::logger::EventLogger logger_;

    // controlMode_ describes the selected control mode (specifies the target motion type)
    ControlMode controlMode_;

    // degrees of freedom of the system
    unsigned int numberOfDOFs_;

    // generator for calling On-Line Trajectory Generation algorithms
    ReflexxesAPI generator_;

    // resultValue_ holds enumeration describing working and error states
    ReflexxesAPI::RMLResultValue resultValue_;

    // vector describing the dimension selected for control
    std::vector<bool> dimVector_;

    // describes the current status of the trajectory generator
    ControlStatus status_;

    utility::types::TaskTrajectoryData trajPoint_;
    mutable std::mutex trajPointMutex_;

    // ===============================================================================================
    // following variables are needed by the ReflexxesAPI in the selected control mode
    // we introduce some helper methods using the common base classes independent of mode
    // ===============================================================================================

    RMLPositionOutputParameters posOutputParam_;
    RMLVelocityOutputParameters velOutputParam_;
    RMLPositionInputParameters posInputParam_;
    RMLVelocityInputParameters velInputParam_;
    RMLVelocityFlags velFlags_;
    RMLPositionFlags posFlags_;

    bool computeNewState();
    bool setCurrentState(const utility::types::TaskTrajectoryData& currentState);
    bool checkValidity(const RMLInputParameters& inputParam);
    bool setNewTrajectoryPoint(const RMLInputParameters& inputParam);
    utility::types::TaskTrajectoryData getNewTrajectoryPoint
            (const RMLOutputParameters& outputParam);
};

}  // namespace crf::control::trajectorypointgenerator
