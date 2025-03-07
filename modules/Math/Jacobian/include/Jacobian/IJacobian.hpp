/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>
#include <memory>

#include <nlohmann/json.hpp>

#include "Types/Types.hpp"

using crf::utility::types::JointPositions;

namespace crf {
namespace math {
namespace jacobian {

/**
 * @ingroup group_jacobian
 * @brief Interface class for the Jacobian objects.
 *
 */
class IJacobian {
 public:
    virtual ~IJacobian() = default;

    /**
     * @brief Calculates the numerical values of the Jacobian Matrix for specified joint angles
     *
     * @param qValues contains the joints position values
     * @return Numeric jacobian matrix
     */
    virtual Eigen::MatrixXd evaluate(const JointPositions& qValues) = 0;
    /**
     * @brief It computes the kinematic manipulability parameter for an especific joints position
     *        values
     *
     * @param qValues contains the joints position values
     * @return Kinematic manipulability parameter
     * @return std::nullopt on failure
     */
    virtual double getKinematicManipulability(const JointPositions& qValues) = 0;
    /**
     * @brief It gives the number of rows of the Jacobian matrix
     *
     * @return number of rows
     */
    virtual unsigned int rows() const = 0;
    /**
     * @brief It gives the number of columns of the Jacobian matrix
     *
     * @return number of columns
     */
    virtual unsigned int cols() const = 0;
};

}  // namespace jacobian
}  // namespace math
}  // namespace crf
