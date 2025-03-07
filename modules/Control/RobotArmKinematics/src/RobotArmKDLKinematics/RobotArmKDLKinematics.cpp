/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <TRACInverseKinematics/trac_ik.hpp>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"

#define IK_SOLVER_MAX_TIME 0.005
#define IK_SOLVER_PRECISION 2e-6

namespace crf::control::robotarmkinematics {

RobotArmKDLKinematics::RobotArmKDLKinematics(
    std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> configuration) :
    logger_("RobotArmKDLKinematics"),
    configuration_(configuration),
    lowerLimits_(configuration_->getNumberOfJoints()),
    upperLimits_(configuration_->getNumberOfJoints()) {
    logger_->debug("CTor");

    std::vector<crf::actuators::robotarm::DHParameter> chainVector =
        configuration_->getKinematicChain();
    std::vector<crf::actuators::robotarm::JointLimits> jointLimitsVector =
        configuration_->getJointsConfiguration();

    for (unsigned int i=0; i< configuration_->getNumberOfJoints(); i++) {
        KDL::Joint::JointType type = KDL::Joint::None;
        if (chainVector[i].type == crf::actuators::robotarm::DHParameter::JointType::Rotational) {
            type = KDL::Joint::RotZ;
        } else if (chainVector[i].type == actuators::robotarm::DHParameter::JointType::Linear) {
            type = KDL::Joint::TransZ;
        } else {
            logger_->warn("Using incorrect JointType");
        }
        chain_.addSegment(KDL::Segment(KDL::Joint(type), KDL::Frame::DH(chainVector[i].a,
            chainVector[i].alpha, chainVector[i].d, chainVector[i].theta)));
        lowerLimits_(i) = jointLimitsVector[i].minimumPosition;
        upperLimits_(i) = jointLimitsVector[i].maximumPosition;
    }
}

boost::optional<crf::utility::types::TaskPose>
    RobotArmKDLKinematics::getPositionForwardKinematic(
    const crf::utility::types::JointPositions& jointPositions) {
    logger_->debug("getPositionForwardKinematic");
    if (jointPositions.size() != configuration_->getNumberOfJoints()) {
        logger_->warn("Input joints position has wrong size. Should be {}",
        configuration_->getNumberOfJoints());
        return boost::none;
    }

    if (!fkpossolver_) {
        fkpossolver_ = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(
            new KDL::ChainFkSolverPos_recursive(chain_));
    }

    auto positions = KDL::JntArray(configuration_->getNumberOfJoints());
    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        positions(i) = static_cast<double>(jointPositions[i]);
    }

    KDL::Frame taskpos;
    if (fkpossolver_->JntToCart(positions, taskpos) < 0) {
        logger_->warn("No solution found for Position Forward Kinematics");
        return boost::none;
    }
    Eigen::Vector3d xyzPos({taskpos.p(0), taskpos.p(1), taskpos.p(2)});
    Eigen::Matrix3d rotMatrix;
    rotMatrix << taskpos.M(0, 0), taskpos.M(0, 1), taskpos.M(0, 2),
                 taskpos.M(1, 0), taskpos.M(1, 1), taskpos.M(1, 2),
                 taskpos.M(2, 0), taskpos.M(2, 1), taskpos.M(2, 2);
    return crf::utility::types::TaskPose(xyzPos, rotMatrix);
}

boost::optional<crf::utility::types::TaskVelocity>
    RobotArmKDLKinematics::getVelocityForwardKinematic(
    const crf::utility::types::JointPositions& jointPositions,
    const crf::utility::types::JointVelocities& jointVelocities) {
    logger_->debug("getVelocityForwardKinematic");
    if ((jointPositions.size() != configuration_->getNumberOfJoints()) ||
        (jointVelocities.size() != configuration_->getNumberOfJoints())) {
        logger_->warn("Input joints objects have wrong size. Should be {}",
            configuration_->getNumberOfJoints());
        return boost::none;
    }

    if (!fkvelsolver_) {
        fkvelsolver_ = std::unique_ptr<KDL::ChainFkSolverVel_recursive>(
            new KDL::ChainFkSolverVel_recursive(chain_));
    }

    auto q = KDL::JntArray(configuration_->getNumberOfJoints());
    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        q(i) = static_cast<double>(jointPositions[i]);
    }

    auto qdot = KDL::JntArray(configuration_->getNumberOfJoints());
    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        qdot(i) = static_cast<double>(jointVelocities[i]);
    }

    KDL::JntArrayVel q_in(q, qdot);
    KDL::FrameVel taskvel;

    if (fkvelsolver_->JntToCart(q_in, taskvel) < 0) {
        logger_->warn("No solution found for Velocity Forward Kinematic");
        return boost::none;
    }

    crf::utility::types::TaskVelocity taskVelocity;

    for (unsigned int i = 3; i < 6; i++) {
        taskVelocity[i] = taskvel.M.w(i-3);
    }
    for (unsigned int i = 0; i < 3; i++) {
        taskVelocity[i] = taskvel.p.v(i);
    }

    return taskVelocity;
}

std::vector<crf::utility::types::JointPositions>
    RobotArmKDLKinematics::getPositionInverseKinematic(
    const crf::utility::types::TaskPose& frame,
    const crf::utility::types::JointPositions& currentJointPositions) {
    logger_->debug("getPositionInverseKinematic");
    if (currentJointPositions.size() != configuration_->getNumberOfJoints()) {
        logger_->warn("Input joints position has wrong size. Should be {}",
            configuration_->getNumberOfJoints());
        return std::vector<crf::utility::types::JointPositions>();
    }

    if (!ikpossolver_) {
        ikpossolver_ = std::unique_ptr<TRAC_IK::TRAC_IK>(
            new TRAC_IK::TRAC_IK(chain_, lowerLimits_, upperLimits_, IK_SOLVER_MAX_TIME,
                IK_SOLVER_PRECISION, TRAC_IK::SolveType::Distance));
    }

    auto initJointPositions = KDL::JntArray(configuration_->getNumberOfJoints());
    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        initJointPositions(i) = static_cast<double>(currentJointPositions[i]);
    }

    auto finalJointPositions = KDL::JntArray(configuration_->getNumberOfJoints());
    auto xyz = frame.getPosition();
    auto rpy = frame.getCardanXYZ();

    KDL::Frame kdl_frame(
        KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]),
        KDL::Vector(xyz(0), xyz(1), xyz(2)));

    int returnValue = ikpossolver_->CartToJnt(initJointPositions, kdl_frame, finalJointPositions);
    if (returnValue < 0) {
        logger_->warn("No solution found for Position Inverse Kinematic");
        return std::vector<crf::utility::types::JointPositions>();
    }

    std::vector<KDL::JntArray> solutionsKDL;
    ikpossolver_->getSolutions(solutionsKDL);

    std::vector<crf::utility::types::JointPositions> solutions;
    for (int j = 0; j < returnValue; ++j) {
        crf::utility::types::JointPositions tempJp(configuration_->getNumberOfJoints());
        for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); ++i) {
            tempJp[i] = solutionsKDL.at(j)(i);
        }
        solutions.push_back(tempJp);
    }

    return solutions;
}

boost::optional<crf::utility::types::JointVelocities>
    RobotArmKDLKinematics::getVelocityInverseKinematic(
    const crf::utility::types::TaskVelocity& targetTaskVelocity,
    const crf::utility::types::JointPositions& currentJointPositions) {
    logger_->debug("getVelocityInverseKinematic");
    if (currentJointPositions.size() != configuration_->getNumberOfJoints()) {
        logger_->warn("Input joints position has wrong size. Should be {}",
            configuration_->getNumberOfJoints());
        return boost::none;
    }

    if (!ikvelsolver_) {
        ikvelsolver_ = std::unique_ptr<KDL::ChainIkSolverVel_pinv>(
            new KDL::ChainIkSolverVel_pinv(chain_));
    }

    auto q_in = KDL::JntArray(configuration_->getNumberOfJoints());

    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        q_in(i) = static_cast<double>(currentJointPositions[i]);
    }

    KDL::Twist twist;
    for (unsigned int i = 0; i < 3; i++) {
        twist.vel[i] = targetTaskVelocity[i];
        twist.rot[i] = targetTaskVelocity[i+3];
    }

    auto finalJointPositions = KDL::JntArray(configuration_->getNumberOfJoints());

    if (ikvelsolver_->CartToJnt(q_in, twist, finalJointPositions) < 0) {
        logger_->warn("No solution found for Velocity Inverse Kinematic");
        return boost::none;
    }

    crf::utility::types::JointVelocities targetJointVelocities(configuration_->getNumberOfJoints());
    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        targetJointVelocities[i] = static_cast<float>(finalJointPositions(i));
    }

    return targetJointVelocities;
}

boost::optional<float> RobotArmKDLKinematics::getManipulability(
    const crf::utility::types::JointPositions& position) {
    logger_->debug("getManipulability");
    if (position.size() != configuration_->getNumberOfJoints()) {
        logger_->warn("Input joints position has wrong size. Should be {}",
            configuration_->getNumberOfJoints());
        return boost::none;
    }

    if (!jacobiansolver_) {
        jacobiansolver_ = std::unique_ptr<KDL::ChainJntToJacSolver>(
            new KDL::ChainJntToJacSolver(chain_));
    }

    auto initJointPositions = KDL::JntArray(configuration_->getNumberOfJoints());
    for (unsigned int i = 0; i < configuration_->getNumberOfJoints(); i++) {
        initJointPositions(i) = static_cast<double>(position[i]);
    }

    KDL::Jacobian jac(configuration_->getNumberOfJoints());
    if (jacobiansolver_->JntToJac(initJointPositions, jac) < 0) {
        logger_->warn("No solution found for Jacobian Solver");
        return boost::none;
    }

    // if it is NaN (not invertible jacobian) we return 0
    double manipulability = sqrt((jac.data * jac.data.transpose()).determinant());
    if (std::isnan(manipulability)) {
        manipulability = 0;
    }

    return static_cast<float>(manipulability);
}

}  // namespace crf::control::robotarmkinematics
