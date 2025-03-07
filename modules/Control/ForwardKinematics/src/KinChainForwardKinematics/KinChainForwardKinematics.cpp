/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO 2023
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
*/

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "ForwardKinematics/KinChainForwardKinematics/KinChainForwardKinematics.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;

namespace crf::control::forwardkinematics {

KinChainForwardKinematics::KinChainForwardKinematics(
    const std::shared_ptr<math::kinematicchain::IKinematicChain>& kinChain) :
    logger_("KinChainForwardKinematics"),
    kinChain_(kinChain) {
    logger_->debug("CTor");

    if (kinChain_->getType() == math::kinematicchain::DescriptionType::Platform ||
        kinChain_->getType() == math::kinematicchain::DescriptionType::Combined) {
        logger_->info(
            "Kinematic chain connected to platform.\n",
            "Forward kinematics will be relative to base!");
    } else {
        logger_->info(
            "Kinematic chain not connected to platform.\n",
            "Calculations will be standard serial arm forward kinematics");
    }
}

KinChainForwardKinematics::~KinChainForwardKinematics() {
    logger_->debug("DTor");
}

std::optional<TaskPose> KinChainForwardKinematics::getPose(const JointPositions& jointPositions) {
    logger_->debug("getPosition");

    if (jointPositions.size() != kinChain_->getChainSize() + kinChain_->getNumWheels()) {
        logger_->error(
            "The dimension of the joints position ({}) does not match ({}).",
            jointPositions.size(),
            kinChain_->getChainSize() + kinChain_->getNumWheels());
        return std::nullopt;
    }

    Eigen::Vector3d IrIE =
        kinChain_->computeTranslation(math::kinematicchain::Translations::IIE, 0, jointPositions);

    Eigen::Quaterniond IRIE =
        kinChain_->computeRotation(math::kinematicchain::Rotations::IIE, 0, jointPositions);

    return TaskPose(IrIE, IRIE);
}

std::optional<TaskVelocity> KinChainForwardKinematics::getVelocity(
    const crf::utility::types::JointPositions& jointPositions,
    const crf::utility::types::JointVelocities& jointVelocities) {
    logger_->error("KinChainForwardKinematics::getVelocity: Not implemented for URDF");
    return std::nullopt;
}

std::optional<TaskAcceleration> KinChainForwardKinematics::getAcceleration(
    const crf::utility::types::JointPositions& jointPositions,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->error("KinChainForwardKinematics::getAcceleration: Not implemented for URDF");
    return std::nullopt;
}

}  // namespace crf::control::forwardkinematics
