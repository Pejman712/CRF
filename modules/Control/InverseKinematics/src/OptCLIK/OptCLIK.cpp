/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include <Eigen/Dense>
#include <optional>
#include <vector>

#include "InverseKinematics/OptCLIK/OptCLIK.hpp"

namespace crf::control::inversekinematics {

OptCLIK::OptCLIK(
    JointPositions qInitial,
    std::chrono::microseconds time,
    std::shared_ptr<crf::actuators::robot::RobotConfiguration> robotArmConf,
    std::vector<double> diagW,
    std::vector<std::shared_ptr<crf::control::inversekinematics::IKinematicObjectiveFunction>>
        objFun,  // NOLINT
    TaskPose tolerance,
    double K,
    double kinManip0,
    double alpha0) :
    logger_("OptCLIK"),
    qInitial_(qInitial),
    time_(time),
    K_(K) {
    logger_->debug("CTor");

    // This time is the control loop time? we can get it from the config
    if (time_ <= std::chrono::microseconds(0)) {
        throw std::invalid_argument("The time cannot be 0 or negative");
    }
    if (K_ <= 0.0) {
        throw std::invalid_argument("The input parameters K, kinManip0 and alpha0 cannot be 0 or "
            "negative");
    }

    forwardKinematics_ = robotArmConf->getForwardKinematics();
    numberOfJoints_ = robotArmConf->getJointSpaceDoF();
    taskSpace_ = robotArmConf->getTaskSpace();

    if (diagW.size() != numberOfJoints_) {
        throw std::invalid_argument("Size of the W diagonal is not equal to number of joints");
    }

    std::optional<TaskPose> positionFK = forwardKinematics_->getPose(qInitial);
    if (!positionFK) {
        throw std::invalid_argument("RIE/quaternion or IrIE have not been read from the JSON file");
    }
    zErrorTolerance_ =
        crf::math::distancemeasures::byQuaternion(TaskPose(), tolerance);

    optOLIK_.reset(new crf::control::inversekinematics::OptOLIK(
        robotArmConf, diagW, objFun, kinManip0, alpha0));
}

OptCLIK::~OptCLIK() {
    logger_->debug("DTor");
}

std::tuple<JointPositions, JointVelocities, JointAccelerations, ResultFlags> OptCLIK::getResults(
    const JointPositions& qAttr,
    const TaskPose& z,
    const TaskVelocity& zd,
    const TaskAcceleration& zdd) {
    logger_->debug("getResults");

    ResultsIK extendedResult(getExtendedResults(qAttr, z, zd));

    return std::make_tuple(
        extendedResult.qResult(),
        extendedResult.qdResult(),
        extendedResult.qddResult(),
        extendedResult.flag());
}

ResultsIK OptCLIK::getExtendedResults(
    const JointPositions& qAttr,
    const TaskPose& z,
    const TaskVelocity& zd,
    const TaskAcceleration& zdd) {
    logger_->debug("getExtendedResults");
    if (qAttr.size() != numberOfJoints_) {
        throw std::invalid_argument("getExtendedResults(): the dimension of qAttr is different "
            "than the length vectors defined in the constructor");
    }
    JointPositions q = qInitial_;
    // zCurrent can't be false. It is tested in the OptCLIK constructor
    TaskPose zCurrent = forwardKinematics_->getPose(q).value();
    Eigen::Vector<double, 6> zError;
    if (taskSpace_.angularDimension() == 1) {
        zError = crf::math::distancemeasures::byCardanXYZ(zCurrent, z);
    } else {
        zError = crf::math::distancemeasures::byQuaternion(zCurrent, z);
        // In the previous version, OptCLIK was using the distance measure below.
        // Because of that, results on intermediate iterations may differ from the ones from before.
        // zError = crf::math::distancemeasures::byRotationMatrix(zCurrent, z);
    }
    TaskVelocity zdClosedLoop(zd.raw() + K_ * zError);

    ResultsIK extendedResultOL(optOLIK_->getExtendedResults(qAttr, q, TaskPose(), zdClosedLoop));
    for (unsigned int i = 0; i < numberOfJoints_; i++) {
        // Euler integration method
        q[i] = q[i] + extendedResultOL.qdResult()[i] * time_.count();
    }
    qInitial_ = q;
    bool isZDesiredAchieved = true;
    for (unsigned int i = 0; i < 3; i++) {
        if (taskSpace_[i] && (abs(zError(i)) > abs(zErrorTolerance_(i)))) {
            isZDesiredAchieved = false;
            break;
        }
    }
    size_t angularDimension = taskSpace_.angularDimension();
    for (unsigned int i = 3; i < 6; i++) {
        if (angularDimension == 1) {
            if (taskSpace_[i] && (abs(zError(i)) > abs(zErrorTolerance_(i)))) {
                isZDesiredAchieved = false;
                break;
            }
        }
        if (angularDimension != 1) {
            if ((abs(zError(i)) > abs(zErrorTolerance_(i)))) {
                isZDesiredAchieved = false;
                break;
            }
        }
    }
    ResultsIK extendedResultCL = extendedResultOL;
    extendedResultCL.zDesired(z);
    extendedResultCL.zdDesired(zd);
    extendedResultCL.qResult(q);
    std::vector<double> zErrorStdVector(6);
    for (size_t i = 0; i < 6; i++) {
        zErrorStdVector[i] = zError(i);
    }
    extendedResultCL.zError(zErrorStdVector);
    if (!isZDesiredAchieved && extendedResultOL.flag() != ResultFlags::workspaceViolation) {
        logger_->error("The desired end-effector position has not been achieved inside the "
            "desired position tolerance");
        extendedResultCL.flag(inversekinematics::ResultFlags::endEffectorToleranceViolation);
    }
    return extendedResultCL;
}

void OptCLIK::updateInitialJointPositions(const JointPositions& qActual) {
    logger_->debug("updateInitialJointPositions");
    if (qActual.size() != numberOfJoints_) {
        throw std::invalid_argument("updateInitialJointPositions(): the dimension of qActual is "
            "different than the length vectors defined in the constructor");
    }
    qInitial_ = qActual;
}

}  // namespace crf::control::inversekinematics
