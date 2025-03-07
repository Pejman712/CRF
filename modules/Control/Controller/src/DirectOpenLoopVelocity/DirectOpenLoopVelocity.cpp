/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *          Alejandro Diaz Rosales CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>
#include <optional>
#include <vector>

#include "Controller/DirectOpenLoopVelocity/DirectOpenLoopVelocity.hpp"

using crf::control::inversekinematics::ResultsIK;
using crf::control::inversekinematics::ResultFlags;

namespace crf::control::controller {

DirectOpenLoopVelocity::DirectOpenLoopVelocity(
    const uint64_t& dimensions,
    std::shared_ptr<IOpenLoopInverseKinematics> inverseKinematics,
    const std::vector<std::shared_ptr<IKinematicObjectiveFunction>>& objectiveFunctions):
    dimensions_(dimensions),
    inverseKinematics_(inverseKinematics),
    objectiveFunctions_(objectiveFunctions),
    result_(),
    qAttraction_(dimensions_),
    emptyDesired0thDer_(TaskPose()),
    resultsIK_(),
    stopCondition_(false),
    stopOnLowManipulability_(false),
    stopOnToleranceViolation_(false),
    stopOnComputationTimeViolation_(false),
    stopOnWorkspaceViolation_(false),
    logger_("DirectOpenLoopVelocity") {
    reset();
}

Signals DirectOpenLoopVelocity::calculate(const JointSignals& desired,
    const JointSignals& actualJoints, const TaskSignals& actualTask) {
    // Check for mandatory parameters
    if (!desired.velocities) {
        stopCondition_ = true;
        throw std::runtime_error("Missing signal values for the DirectOpenLoopVelocity control");
    }
    if (desired.velocities.value().size() != dimensions_ ||
        actualJoints.velocities.value().size() != dimensions_) {
        stopCondition_ = true;
        throw std::runtime_error("Input values don't have the correct sizes for the control");
    }
    result_.joints.positions = crf::Code::Empty;
    result_.joints.velocities = desired.velocities.value();
    result_.joints.accelerations = crf::Code::Empty;
    result_.joints.forceTorques = crf::Code::Empty;
    result_.task.pose = crf::Code::Empty;
    result_.task.velocity = crf::Code::Empty;
    result_.task.acceleration = crf::Code::Empty;
    result_.task.forceTorque = crf::Code::Empty;
    stopCondition_ = false;
    return result_;
}

Signals DirectOpenLoopVelocity::calculate(const TaskSignals& desired,
    const JointSignals& actualJoints, const TaskSignals& actualTask) {
    // Check for mandatory parameters
    if (!actualJoints.positions || !desired.velocity) {
        stopCondition_ = true;
        throw std::runtime_error("Missing signal values for the DirectOpenLoopVelocity control");
    }
    if (actualJoints.velocities.value().size() != dimensions_) {
        stopCondition_ = true;
        throw std::runtime_error("Input values don't have the correct sizes for the control");
    }
    // Inverse kinematics
    ResultsIK resultsIK_ = inverseKinematics_->getExtendedResults(qAttraction_,
        actualJoints.positions.value(), emptyDesired0thDer_, desired.velocity.value());
    result_.joints.velocities = resultsIK_.qdResult();
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
    }
    return result_;
}

bool DirectOpenLoopVelocity::checkStopCondition() {
    return stopCondition_;
}

crf::expected<bool> DirectOpenLoopVelocity::setParameters(const nlohmann::json& parameters) {
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
    } catch (std::exception& e ) {
        logger_->error(
            "Error setting parameters for controller \"DirectOpenLoopVelocity\": {}", e.what());
        return crf::Code::BadRequest;
    }
    return true;
}

nlohmann::json DirectOpenLoopVelocity::getParameters() {
    nlohmann::json output;
    output["StopConditions"]["StopOnLowManipulability"] = stopOnLowManipulability_.load();
    output["StopConditions"]["StopOnToleranceViolation"] = stopOnToleranceViolation_.load();
    output["StopConditions"]["StopOnComputationTimeViolation"] = stopOnComputationTimeViolation_.load();  // NOLINT
    output["StopConditions"]["StopOnWorkspaceViolation"] = stopOnWorkspaceViolation_.load();
    output["DesiredJointPosition"]["AttractionPosition"] = qAttraction_;
    return output;
}

void DirectOpenLoopVelocity::reset() {
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
    return;
}

}  // namespace crf::control::controller
