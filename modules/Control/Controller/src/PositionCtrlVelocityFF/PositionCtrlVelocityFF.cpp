/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <optional>
#include <vector>

#include "Controller/PositionCtrlVelocityFF/PositionCtrlVelocityFF.hpp"

using crf::control::inversekinematics::ResultsIK;
using crf::control::inversekinematics::ResultFlags;

namespace crf::control::controller {

PositionCtrlVelocityFF::PositionCtrlVelocityFF(
    const std::vector<double>& Kp,
    const std::vector<double>& Ki,
    const std::vector<double>& Kd,
    const double& Ts,
    std::shared_ptr<IClosedLoopInverseKinematics> inverseKinematics,
    const std::vector<std::shared_ptr<IKinematicObjectiveFunction>>& objectiveFunctions):
    Kp_(Kp),
    Ki_(Ki),
    Kd_(Kd),
    Ts_(Ts),
    inverseKinematics_(inverseKinematics),
    objectiveFunctions_(objectiveFunctions),
    pidResult_(Kp.size()),
    result_(),
    dimensions_(Kp.size()),
    qAttraction_(dimensions_),
    resultsIK_(),
    stopCondition_(false),
    stopOnLowManipulability_(false),
    stopOnToleranceViolation_(false),
    stopOnComputationTimeViolation_(false),
    stopOnWorkspaceViolation_(false),
    logger_("PositionCtrlVelocityFF") {
    if (Kp.size() != Ki.size() && Kp.size() != Kd.size()) {
        throw std::runtime_error("PositionCtrlVelocityFF - Controller dimensions don't match");
    }
    reset();
}

Signals PositionCtrlVelocityFF::calculate(const JointSignals& desired,
    const JointSignals& actualJoints, const TaskSignals& actualTask) {
    // Check for mandatory parameters
    if (!actualJoints.positions || !desired.positions || !desired.velocities) {
        throw std::runtime_error("Missing signal values for the PositionCtrlVelocityFF control");
    }
    if ((actualJoints.positions.value().size() != dimensions_) ||
        (desired.positions.value().size() != dimensions_) ||
        (desired.velocities.value().size() != dimensions_)) {
        throw std::runtime_error("Input values don't have the correct sizes for the control");
    }
    std::vector<double> desiredPositions(stdVectorFromEigenVector(
        desired.positions.value().raw()));
    std::vector<double> actualPositions(stdVectorFromEigenVector(
        actualJoints.positions.value().raw()));

    // Controller
    pidResult_ = PID(desiredPositions, actualPositions, pidResult_.error, Kp_, Ki_, Kd_, Ts_);
    for (uint64_t i = 0; i < dimensions_; i++) {
        result_.joints.velocities.value()[i] = pidResult_.output[i] + desired.velocities.value()[i];
    }
    result_.joints.positions = crf::Code::Empty;
    result_.joints.accelerations = crf::Code::Empty;
    result_.joints.forceTorques = crf::Code::Empty;
    result_.task.pose = crf::Code::Empty;
    result_.task.velocity = crf::Code::Empty;
    result_.task.acceleration = crf::Code::Empty;
    result_.task.forceTorque = crf::Code::Empty;
    stopCondition_ = false;
    return result_;
}

Signals PositionCtrlVelocityFF::calculate(const TaskSignals& desired,
    const JointSignals& actualJoints, const TaskSignals& actualTask) {
    // Check for mandatory parameters
    if (!desired.pose) {
        throw std::runtime_error("Missing signal values for the PositionCtrlVelocityFF control");
    }
    // Inverse kinematics
    JointSignals desiredJoint;
    if (!desired.velocity) {
        resultsIK_ = inverseKinematics_->getExtendedResults(qAttraction_,
            desired.pose.value());
        desiredJoint.positions = resultsIK_.qResult();
    } else if (!desired.acceleration) {
        resultsIK_ = inverseKinematics_->getExtendedResults(qAttraction_,
            desired.pose.value(), desired.velocity.value());
        desiredJoint.positions = resultsIK_.qResult();
        desiredJoint.velocities = resultsIK_.qdResult();
    } else {
        resultsIK_ = inverseKinematics_->getExtendedResults(qAttraction_,
        desired.pose.value(), desired.velocity.value(), desired.acceleration.value());
        desiredJoint.positions = resultsIK_.qResult();
        desiredJoint.velocities = resultsIK_.qdResult();
        // TODO(any): The IK returns a Joint acc of size 1 when it's not used
        if (!areAlmostEqual(resultsIK_.qddResult(), JointAccelerations(1))) {
            desiredJoint.accelerations = resultsIK_.qddResult();
        }
    }
    stopCondition_ = false;
    // Check inverse kinematics flag
    if (resultsIK_.flag() == ResultFlags::notDefined) {
        stopCondition_ = true;
        throw std::runtime_error("Undefined inverse kinematics result");
    } else if (resultsIK_.flag() == ResultFlags::lowManipulability && stopOnLowManipulability_) {
        stopCondition_ = true;
        result_.joints.positions = crf::Code::LowManipulability;
        result_.joints.velocities = crf::Code::LowManipulability;
        result_.joints.accelerations = crf::Code::LowManipulability;
        result_.joints.forceTorques = crf::Code::LowManipulability;
        result_.task.pose = crf::Code::LowManipulability;
        result_.task.velocity = crf::Code::LowManipulability;
        result_.task.acceleration = crf::Code::LowManipulability;
        result_.task.forceTorque = crf::Code::LowManipulability;
    } else if (resultsIK_.flag() == ResultFlags::endEffectorToleranceViolation && stopOnToleranceViolation_) {  // NOLINT
        stopCondition_ = true;
        result_.joints.positions = crf::Code::EndEffectorToleranceViolation;
        result_.joints.velocities = crf::Code::EndEffectorToleranceViolation;
        result_.joints.accelerations = crf::Code::EndEffectorToleranceViolation;
        result_.joints.forceTorques = crf::Code::EndEffectorToleranceViolation;
        result_.task.pose = crf::Code::EndEffectorToleranceViolation;
        result_.task.velocity = crf::Code::EndEffectorToleranceViolation;
        result_.task.acceleration = crf::Code::EndEffectorToleranceViolation;
        result_.task.forceTorque = crf::Code::EndEffectorToleranceViolation;
    } else if (resultsIK_.flag() == ResultFlags::maxComputationTimeViolation && stopOnComputationTimeViolation_) {  // NOLINT
        stopCondition_ = true;
        result_.joints.positions = crf::Code::RealTimeViolation;
        result_.joints.velocities = crf::Code::RealTimeViolation;
        result_.joints.accelerations = crf::Code::RealTimeViolation;
        result_.joints.forceTorques = crf::Code::RealTimeViolation;
        result_.task.pose = crf::Code::RealTimeViolation;
        result_.task.velocity = crf::Code::RealTimeViolation;
        result_.task.acceleration = crf::Code::RealTimeViolation;
        result_.task.forceTorque = crf::Code::RealTimeViolation;
    } else if (resultsIK_.flag() == ResultFlags::workspaceViolation && stopOnWorkspaceViolation_) {
        stopCondition_ = true;
        result_.joints.positions = crf::Code::WorkspaceViolation;
        result_.joints.velocities = crf::Code::WorkspaceViolation;
        result_.joints.accelerations = crf::Code::WorkspaceViolation;
        result_.joints.forceTorques = crf::Code::WorkspaceViolation;
        result_.task.pose = crf::Code::WorkspaceViolation;
        result_.task.velocity = crf::Code::WorkspaceViolation;
        result_.task.acceleration = crf::Code::WorkspaceViolation;
        result_.task.forceTorque = crf::Code::WorkspaceViolation;
    } else {
        return calculate(desiredJoint, actualJoints, actualTask);
    }
    return result_;
}

bool PositionCtrlVelocityFF::checkStopCondition() {
    return stopCondition_;
}

crf::expected<bool> PositionCtrlVelocityFF::setParameters(const nlohmann::json& parameters) {
    try {
        if (parameters.contains("StopConditions")) {
            if (parameters["StopConditions"].contains("StopOnLowManipulability")) {
                stopOnLowManipulability_ = parameters["StopOnLowManipulability"].get<bool>();
            }
            if (parameters["StopConditions"].contains("StopOnToleranceViolation")) {
                stopOnToleranceViolation_ = parameters["StopOnToleranceViolation"].get<bool>();
            }
            if (parameters["StopConditions"].contains("StopOnComputationTimeViolation")) {
                stopOnComputationTimeViolation_ =
                    parameters["StopOnComputationTimeViolation"].get<bool>();
            }
            if (parameters["StopConditions"].contains("StopOnWorkspaceViolation")) {
                stopOnWorkspaceViolation_ = parameters["StopOnWorkspaceViolation"].get<bool>();
            }
        }
        if (parameters.contains("DesiredJointPosition")) {
            if (parameters["DesiredJointPosition"].contains("AttractionPosition")) {
                for (uint64_t i = 0; i < dimensions_; i++) {
                    auto& position = parameters["DesiredJointPosition"]["AttractionPosition"][i];
                    if (position == "NaN") {
                        qAttraction_[i] = std::nan("");
                    } else {
                        qAttraction_[i] = position.get<double>();
                    }
                }
            }
        }
        if (parameters.contains("UpdateJointsInitialPosition")) {
            inverseKinematics_->updateInitialJointPositions(
                parameters["UpdateJointsInitialPosition"].get<JointPositions>());
        }
    } catch (std::exception& e ) {
        logger_->error(
            "Error setting parameters for controller \"DirectOpenLoopVelocity\": {}", e.what());
        return crf::Code::BadRequest;
    }
    return true;
}

nlohmann::json PositionCtrlVelocityFF::getParameters() {
    nlohmann::json output;
    output["StopConditions"]["StopOnLowManipulability"] = stopOnLowManipulability_.load();
    output["StopConditions"]["StopOnToleranceViolation"] = stopOnToleranceViolation_.load();
    output["StopConditions"]["StopOnComputationTimeViolation"] = stopOnComputationTimeViolation_.load();  // NOLINT
    output["StopConditions"]["StopOnWorkspaceViolation"] = stopOnWorkspaceViolation_.load();
    output["DesiredJointPosition"]["AttractionPosition"] = qAttraction_;
    return output;
}

void PositionCtrlVelocityFF::reset() {
    pidResult_ = PIDResult(dimensions_);
    result_.joints.positions = crf::Code::Empty;
    result_.joints.velocities = std::vector<double>(dimensions_, 0);
    result_.joints.accelerations = crf::Code::Empty;
    result_.joints.forceTorques = crf::Code::Empty;
    result_.task.pose = crf::Code::Empty;
    result_.task.velocity = crf::Code::Empty;
    result_.task.acceleration = crf::Code::Empty;
    result_.task.forceTorque = crf::Code::Empty;
    qAttraction_ = JointPositions(std::vector(dimensions_, std::nan("")));
    stopCondition_ = false;
    stopOnLowManipulability_ = false;
    stopOnToleranceViolation_ = false;
    stopOnComputationTimeViolation_ = false;
    stopOnWorkspaceViolation_ = false;
}

}  // namespace crf::control::controller
