/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <memory>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <TRACInverseKinematics/trac_ik.hpp>

#include "EventLogger/EventLogger.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "RobotArmKinematics/IRobotArmKinematics.hpp"
#include "Types/Types.hpp"

namespace crf::control::robotarmkinematics {

class RobotArmKDLKinematics : public IRobotArmKinematics {
 public:
    RobotArmKDLKinematics() = delete;
    explicit RobotArmKDLKinematics(
        std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> configuration);
    ~RobotArmKDLKinematics() override = default;

    boost::optional<crf::utility::types::TaskPose> getPositionForwardKinematic(
        const crf::utility::types::JointPositions& jointPositions) override;
    boost::optional<crf::utility::types::TaskVelocity> getVelocityForwardKinematic(
        const crf::utility::types::JointPositions& jointPositions,
        const crf::utility::types::JointVelocities& jointVelocities) override;
    std::vector<crf::utility::types::JointPositions> getPositionInverseKinematic(
        const crf::utility::types::TaskPose& frame,
        const crf::utility::types::JointPositions& currentJointPositions) override;
    boost::optional<crf::utility::types::JointVelocities> getVelocityInverseKinematic(
        const crf::utility::types::TaskVelocity& targetTaskVelocity,
        const crf::utility::types::JointPositions& currentJointPositions) override;
    boost::optional<float> getManipulability(
        const crf::utility::types::JointPositions& position) override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> configuration_;

    KDL::Chain chain_;
    KDL::JntArray lowerLimits_;
    KDL::JntArray upperLimits_;

    // They are lazily initialized at the first call of their function.
    std::unique_ptr<TRAC_IK::TRAC_IK> ikpossolver_;
    std::unique_ptr<KDL::ChainIkSolverVel> ikvelsolver_;
    std::unique_ptr<KDL::ChainFkSolverPos> fkpossolver_;
    std::unique_ptr<KDL::ChainFkSolverVel> fkvelsolver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobiansolver_;
};

}  // namespace crf::control::robotarmkinematics
