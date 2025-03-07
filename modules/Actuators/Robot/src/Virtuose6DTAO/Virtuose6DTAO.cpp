/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <set>
#include <vector>

#include <nlohmann/json.hpp>
#include <Eigen/Dense>

#include "crf/expected.hpp"
#include "Types/Types.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "Robot/Virtuose6DTAO/Virtuose6DTAO.hpp"
#include "Robot/Virtuose6DTAO/Virtuose6DTAOConfiguration.hpp"

namespace crf::actuators::robot {

Virtuose6DTAO::Virtuose6DTAO(std::shared_ptr<devices::haption::IHaptionAPI> haptionInterface,
    const Virtuose6DTAOConfiguration& configuration):
    haptionInterface_(haptionInterface),
    configuration_(configuration),
    logger_("Virtuose6DTAO"),
    isInitialized_(false) {
    logger_->debug("CTor");
}

Virtuose6DTAO::~Virtuose6DTAO() {
    logger_->debug("DTor");
}

bool Virtuose6DTAO::initialize() {
    logger_->debug("initialize");

    if (isInitialized_) {
        logger_->error("The robot is already initialized");
        return true;
    }
    crf::Code result = haptionInterface_->startConnection();
    if (result != crf::Code::OK) {
        logger_->error("Failed to initialize the haption device");
        return false;
    }
    logger_->info("Configuration file loaded");
    isInitialized_ = true;
    return true;
}

bool Virtuose6DTAO::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->error("The robot is not initialized");
        return true;
    }
    crf::Code result = haptionInterface_->stopConnection();
    if (result != crf::Code::OK) {
        logger_->error("Failed to deinitialize the haption device");
        return false;
    }
    isInitialized_ = false;
    return true;
}

crf::expected<crf::utility::types::JointPositions> Virtuose6DTAO::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    HAPTION::JointVector p;
    crf::Code result = haptionInterface_->getJointAngles(p);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the joint positions from the haption device");
        return result;
    }
    return crf::utility::types::JointPositions(
        {p.v[0], p.v[1], p.v[2], p.v[3], p.v[4], p.v[5], p.v[6]});
}

crf::expected<crf::utility::types::JointVelocities> Virtuose6DTAO::getJointVelocities() {
    logger_->debug("getJointVelocities");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    HAPTION::JointVector v;
    crf::Code result = haptionInterface_->getJointSpeeds(v);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the joint velocities from  the haption device");
        return result;
    }
    return crf::utility::types::JointVelocities(
        {v.v[0], v.v[1], v.v[2], v.v[3], v.v[4], v.v[5], v.v[6]});
}

crf::expected<crf::utility::types::JointAccelerations> Virtuose6DTAO::getJointAccelerations() {
    logger_->debug("getJointAccelerations");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointForceTorques> Virtuose6DTAO::getJointForceTorques() {
    logger_->debug("getJointForceTorques");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    HAPTION::JointVector t;
    crf::Code result = haptionInterface_->getJointTorques(t);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the joint force torques from the haption device");
        return result;
    }
    return crf::utility::types::JointForceTorques(
        {t.v[0], t.v[1], t.v[2], t.v[3], t.v[4], t.v[5], t.v[6]});
}

crf::expected<crf::utility::types::TaskPose> Virtuose6DTAO::getTaskPose() {
    logger_->debug("getTaskPose");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    HAPTION::Displacement p;
    crf::Code result = haptionInterface_->getCartesianPose(p);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the task pose from the haption device");
        return result;
    }
    return crf::utility::types::TaskPose(Eigen::Vector3d(p.t_x, p.t_y, p.t_z),
        Eigen::Quaterniond(p.q_w, p.q_x, p.q_y, p.q_z), quaternionAccuracy_);
}

crf::expected<crf::utility::types::TaskVelocity> Virtuose6DTAO::getTaskVelocity() {
    logger_->debug("getTaskVelocity");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    HAPTION::CartesianVector v;
    crf::Code result = haptionInterface_->getCartesianSpeed(v);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the task velocity from the haption device");
        return result;
    }
    return crf::utility::types::TaskVelocity({v.t_x, v.t_y, v.t_z, v.r_x, v.r_y, v.r_z});
}

crf::expected<crf::utility::types::TaskAcceleration> Virtuose6DTAO::getTaskAcceleration() {
    logger_->debug("getTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskForceTorque> Virtuose6DTAO::getTaskForceTorque() {
    logger_->debug("getTaskForceTorque");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    crf::Code result = haptionInterface_->startCartesianPositionMode();
    if (result != crf::Code::OK) {
        logger_->error("Failed to start the cartesian mode");
        return result;
    }
    HAPTION::CartesianVector f;
    result = haptionInterface_->getCartesianForce(f);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the task force torque from the haption device");
        return result;
    }
    return crf::utility::types::TaskForceTorque({f.t_x, f.t_y, f.t_z, f.r_x, f.r_y, f.r_z});
}

crf::expected<bool> Virtuose6DTAO::setJointPositions(const bool& isSmoothTrajectory,
    const crf::utility::types::JointPositions& jointPositions,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointPositions");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setJointVelocities(const bool& isSmoothTrajectory,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointVelocities");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setJointForceTorques(const bool& isSmoothTrajectory,
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->debug("setJointForceTorques");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    crf::Code result = haptionInterface_->startJointPositionMode();
    if (result != crf::Code::OK) {
        logger_->error("Failed to start the joint mode");
        return result;
    }
    // We need to send with SetJointAngles and SetJointSpeeds the current pose and speed of the
    // device (as given by GetJointAngles and GetJointSpeeds), otherwise the software will give
    // a drag error!
    HAPTION::JointVector p;
    HAPTION::JointVector v;
    result = haptionInterface_->getJointAngles(p);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the joint positions from the haption device");
        return result;
    }
    result = haptionInterface_->getJointSpeeds(v);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the joint velocities from the haption device");
        return result;
    }
    result = haptionInterface_->setJointAngles(v);
    if (result != crf::Code::OK) {
        logger_->error("Failed to set the joint positions to the haption device");
        return result;
    }
    result = haptionInterface_->setJointSpeeds(v);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the joint velocities to the haption device");
        return result;
    }
    HAPTION::JointVector t;
    t.v[0] = jointForceTorques[0];
    t.v[1] = jointForceTorques[1];
    t.v[2] = jointForceTorques[2];
    t.v[3] = jointForceTorques[3];
    t.v[4] = jointForceTorques[4];
    t.v[5] = jointForceTorques[5];
    t.v[6] = jointForceTorques[6];
    return haptionInterface_->addJointTorqueOverlay(t);
}

crf::expected<bool> Virtuose6DTAO::setTaskPose(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskPose& taskPose,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskPose");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setTaskVelocity(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setTaskForceTorque(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskForceTorque& taskForceTorque) {
    logger_->debug("setTaskForceTorque");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    crf::Code result = haptionInterface_->startCartesianPositionMode();
    if (result != crf::Code::OK) {
        logger_->error("Failed to start the cartesian mode");
        return result;
    }
    // We need to send with SetCartesianPose and SetCartesianSpeed the current pose and speed of
    // the device (as given by GetCartesianPose and GetCartesianSpeed), otherwise the software will
    // give a drag error!
    HAPTION::Displacement p;
    HAPTION::CartesianVector v;
    result = haptionInterface_->getCartesianPose(p);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the task pose from the haption device");
        return result;
    }
    result = haptionInterface_->getCartesianSpeed(v);
    if (result != crf::Code::OK) {
        logger_->error("Failed to get the task velocity from the haption device");
        return result;
    }
    result = haptionInterface_->setCartesianPose(p);
    if (result != crf::Code::OK) {
        logger_->error("Failed to set the task pose to the haption device");
        return result;
    }
    result = haptionInterface_->setCartesianSpeed(v);
    if (result != crf::Code::OK) {
        logger_->error("Failed to set the task velocity to the haption device");
        return result;
    }
    HAPTION::CartesianVector t;
    t.t_x = taskForceTorque[0];
    t.t_y = taskForceTorque[1];
    t.t_z = taskForceTorque[2];
    t.r_x = taskForceTorque[3];
    t.r_y = taskForceTorque[4];
    t.r_z = taskForceTorque[5];
    result = haptionInterface_->addCartesianForceOverlay(t);
    if (result == crf::Code::OK) {
        return true;
    }
    return result;
}

crf::expected<crf::utility::types::JointVelocities> Virtuose6DTAO::getProfileJointVelocities() {  // NOLINT
    logger_->debug("getProfileJointVelocities");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointAccelerations> Virtuose6DTAO::getProfileJointAccelerations() {  // NOLINT
    logger_->debug("getProfileJointAccelerations");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskVelocity> Virtuose6DTAO::getProfileTaskVelocity() {
    logger_->debug("getProfileTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskAcceleration> Virtuose6DTAO::getProfileTaskAcceleration() {  // NOLINT
    logger_->debug("getProfileTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setProfileJointVelocities(
    const crf::utility::types::JointVelocities& jointVelocities) {
    logger_->debug("setProfileJointVelocities");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setProfileJointAccelerations(
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setProfileJointAccelerations");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setProfileTaskVelocity(
    const crf::utility::types::TaskVelocity& taskVelocity) {
    logger_->debug("setProfileTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setProfileTaskAcceleration(
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setProfileTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setGravity(const std::array<double, 3>& gravity) {
    logger_->warn("setGravity not supported");
    return crf::Code::NotImplemented;
}

crf::expected<bool> Virtuose6DTAO::softStop() {
    logger_->debug("softStop");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::hardStop() {
    logger_->debug("hardStop");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Virtuose6DTAO::setBrakes(std::vector<bool> brakesStatus) {
    logger_->debug("setBrakes");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    const uint nDoF = configuration_.getJointSpaceDoF();
    if (brakesStatus.size() != nDoF) {
        logger_->error("Input velocity size not valid");
        return crf::Code::BadRequest;
    }
    if (brakesStatus == std::vector<bool>(configuration_.getJointSpaceDoF(), true)) {
        return haptionInterface_->changeBrakeStatus(HAPTION::BrakeStatus::BRAKE_ENGAGED);
    } else if (brakesStatus == std::vector<bool>(configuration_.getJointSpaceDoF(), false)) {
        return haptionInterface_->changeBrakeStatus(HAPTION::BrakeStatus::BRAKE_RELEASED);
    }
    logger_->error("All brakes have to be engaged or released at the same time");
    return crf::Code::BadRequest;
}

crf::expected<std::vector<bool>> Virtuose6DTAO::getBrakes() {
    logger_->debug("getBrakes");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    HAPTION::BrakeStatus status;
    crf::Code result = haptionInterface_->getBrakeStatus(status);
    if (result != crf::Code::OK) {
        logger_->error("Failed to deinitialize the haption device");
        return result;
    }
    if (status == HAPTION::BrakeStatus::BRAKE_ENGAGED) {
        return std::vector<bool>(configuration_.getJointSpaceDoF(), true);
    }
    return std::vector<bool>(configuration_.getJointSpaceDoF(), false);
}

std::set<crf::Code> Virtuose6DTAO::robotStatus() {
    logger_->debug("robotStatus");
    std::set<Code> status;
    status.insert(haptionInterface_->getErrorStatus());

    crf::Code result;
    HAPTION::PowerStatus powerStatus;
    result = haptionInterface_->getPowerStatus(powerStatus);
    if (result == crf::Code::OK) {
        if (powerStatus == HAPTION::PowerStatus::P_NOPOWER) {
            logger_->warn("No power on the motor H-Bridge.");
            status.insert(crf::Code::PoweredOff);
        } else if (powerStatus == HAPTION::PowerStatus::P_POWER_INHIBITED) {
            logger_->warn("Power on the motor H-Bridge, but H-Bridge closed.");
            status.insert(crf::Code::PoweredOff);
        } else if (powerStatus == HAPTION::PowerStatus::P_POWER_DISINHIBITED) {
            logger_->warn("Power on the motor H-Bridge, and H-Bridge open. ");
            status.insert(crf::Code::PoweredOn);
        } else if (powerStatus == HAPTION::PowerStatus::P_POWER_EMERGENCY_STOP) {
            logger_->warn("Emergency Stop");
            status.insert(crf::Code::EmergencyStop);
        }
    }
    HAPTION::AutomatonStatus automatonStatus;
    result = haptionInterface_->getAutomatonStatus(automatonStatus);
    if (result == crf::Code::OK) {
        if (automatonStatus == HAPTION::AutomatonStatus::A_NONE) {
            logger_->warn("Unknown state, machine not running.");
        } else if (automatonStatus == HAPTION::AutomatonStatus::A_CALIBRATION) {
            logger_->info("Waiting for calibration");
            status.insert(crf::Code::NotCalibrated);
        } else if (automatonStatus == HAPTION::AutomatonStatus::A_IDLE) {
            logger_->info("Waiting for activation command.");
            status.insert(crf::Code::Idle);
        } else if (automatonStatus == HAPTION::AutomatonStatus::A_WAIT_POWER) {
            logger_->info("Waiting for operator to press the power button.");
        } else if (automatonStatus == HAPTION::AutomatonStatus::A_INACTIVE) {
            logger_->info("Waiting for operator to activate the deadman-switch.");
            status.insert(crf::Code::DeadmanSwitchReleased);
        } else if (automatonStatus == HAPTION::AutomatonStatus::A_HOMING) {
            logger_->info("Moving to the start pose.");
        } else if (automatonStatus == HAPTION::AutomatonStatus::A_ERROR) {
            logger_->warn("Non-fatal error, can be solved by depressing the power button.");
        } else if (automatonStatus == HAPTION::AutomatonStatus::A_BREAKDOWN) {
            logger_->info("State machine Fatal error.");
        }
    }
    std::array<HAPTION::MotorStatus, HAPTION::MAX_NB_JOINTS> motor;
    result = haptionInterface_->getMotorStatus(motor);
    if (result == crf::Code::OK) {
        for (uint16_t i = 0; i < HAPTION::MAX_NB_JOINTS; i++) {
            if (motor[0].calibrated == 0) {
                logger_->warn("The actuator {} is not calibrated.", i);
            }
            if (motor[0].inhibition == 1) {
                logger_->error("The PWM of the actuator {} is inhibited.", i);
            }
            if (motor[0].i2t == 1) {
                logger_->error("Reducing torque of actuator {} due to overheat (I2T).", i);
            }
            if (motor[0].encoderError == 1) {
                logger_->error("Breakdown or bus error in the encoder of the actuator {}.", i);
            }
            if (motor[0].currentError == 1) {
                logger_->error("Current drag error in the actuator {}.", i);
            }
            if (motor[0].pwmError == 1) {
                logger_->error("H-Bridge breakdown of the actuator (PWM) {}.", i);
            }
            if (motor[0].pwmOverheatError == 1) {
                logger_->error("H-Bridge overheat of the actuator (PWM) {}.", i);
            }
            if (motor[0].checksumDSPError == 1) {
                logger_->error("Protocol error on internal DSP bus of the actuator {}.", i);
            }
            if (motor[0].checksumCalculatedError == 1) {
                logger_->error("Protocol error on internal bus of the actuator {} controller.", i);
            }
            if (motor[0].encoderMismatchError == 1) {
                logger_->error("Encoder mismatch on the actuator {}.", i);
            }
            if (motor[0].overSpeed == 1) {
                logger_->error("Motor {} speed is higher than limit.", i);
            }
            if (motor[0].connectionError == 1) {
                logger_->error("Motor {} CAN error.", i);
            }
            if (motor[0].mechanicalError == 1) {
                logger_->error("Motor {} spring compensation error.", i);
            }
            if (motor[0].dragError == 1) {
                logger_->error("Motor {} drag error.", i);
            }
            if (motor[0].discontinuityError == 1) {
                logger_->error("Motor {} discontinuity error.", i);
            }
        }
    }
    return status;
}

crf::expected<bool> Virtuose6DTAO::resetFaultState() {
    logger_->debug("resetFaultState");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    return haptionInterface_->clearError();
}

std::shared_ptr<RobotConfiguration> Virtuose6DTAO::getConfiguration() {
    logger_->debug("getConfiguration");
    return std::make_shared<RobotConfiguration>(configuration_);
}

crf::expected<bool> Virtuose6DTAO::setInitialCartesianPose(
    const crf::utility::types::TaskPose& pose) {
    logger_->debug("setInitialCartesianPose");
    crf::Code result = haptionInterface_->startCartesianPositionMode();
    if (result != crf::Code::OK) {
        logger_->error("Failed to start the cartesian mode");
        return result;
    }
    Eigen::Vector3d position = pose.getPosition();
    Eigen::Quaterniond quaternion = pose.getQuaternion();
    HAPTION::Displacement haptionPose{
        static_cast<HAPTION::float32_t>(position[0]),
        static_cast<HAPTION::float32_t>(position[1]),
        static_cast<HAPTION::float32_t>(position[2]),
        static_cast<HAPTION::float32_t>(quaternion.x()),
        static_cast<HAPTION::float32_t>(quaternion.y()),
        static_cast<HAPTION::float32_t>(quaternion.z()),
        static_cast<HAPTION::float32_t>(quaternion.w())};

    result = haptionInterface_->forceCartesianPose(haptionPose);
    if (result != crf::Code::OK) {
        logger_->error("Failed to set the cartesian pose, overwriting the clutch offset");
        return result;
    }
    return true;
}

}  // namespace crf::actuators::robot
