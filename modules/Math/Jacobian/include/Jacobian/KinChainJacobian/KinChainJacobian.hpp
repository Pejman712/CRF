/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Ante Marić CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Jacobian/IJacobian.hpp"
#include "KinematicChain/DescriptionTypes.hpp"
#include "KinematicChain/IKinematicChain.hpp"

using crf::math::kinematicchain::IKinematicChain;
using crf::utility::types::TaskSpace;

namespace crf::math::jacobian {

/**
 * @ingroup group_kinematic_chain_jacobian
 * @brief Implementation of IJacobian. This implementation constructs a Jacobian of
 * a robot with a kinematic chain object provided
 *
 */
class KinChainJacobian : public IJacobian {
 public:
    /**
     * @brief Construct a new Jacobian object from a KinematicChain object
     *
     * @param kinChain is a shared pointer to a KinematicChain object
     */
    explicit KinChainJacobian(std::shared_ptr<IKinematicChain> kinChain,
        const TaskSpace& taskSpace = TaskSpace());

    /**
     * @brief Destroy the Jacobian object
     */
    ~KinChainJacobian() override;

    /**
     * @brief Calculates the numerical values of the Jacobian Matrix for specified joint angles
     *
     * @param qValues contains the joints position values
     * @return Numeric jacobian matrix
     */
    Eigen::MatrixXd evaluate(const JointPositions& qValues) override;
    /**
     * @brief It computes the kinematic manipulability parameter for an especific joints position
     *        values
     *
     * @param qValues contains the joints position values
     * @return Kinematic manipulability parameter
     * @return std::nullopt on failure
     */
    double getKinematicManipulability(const JointPositions& qValues) override;
    /**
     * @brief It gives the number of rows of the Jacobian matrix
     *
     * @return number of rows
     */
    unsigned int rows() const override;
    /**
     * @brief It gives the number of columns of the Jacobian matrix
     *
     * @return number of columns
     */
    unsigned int cols() const override;

 private:
    std::shared_ptr<math::kinematicchain::IKinematicChain> kinChain_;

    unsigned int chainSize_;
    unsigned int numWheels_;
    unsigned int rowsJMatrix_;
    unsigned int columnsJMatrix_;
    Eigen::MatrixXd JMatrix_;
    TaskSpace taskSpace_;

    crf::utility::logger::EventLogger logger_;

    /**
     * @brief Computes part of Jacobian matrix related to a kinematic base with 4 omniwheels
     * @param platformL platform length
     * @param platformW platform width
     * @param wheelRadius wheel radius
     * @return Jplatform - part of Jacobian matrix related to the platform
     */
    Eigen::MatrixXd platformJacobian();
};

}  // namespace crf::math::jacobian
