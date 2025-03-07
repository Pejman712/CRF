/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Chelsea Davidson CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#pragma once

#include "Rotation/Rotation.hpp"

#include <Eigen/Dense>
#include <array>
#include <optional>

using crf::math::rotation::Orientation;

namespace crf {
namespace math {
namespace geometricmethods {

/**
 * @ingroup group_geometric_methods
 * @brief
 */
class IOrientationMethods {
 public:
    virtual ~IOrientationMethods() = default;
    /**
     * @brief It calculates the difference between the last evaluation point of the cure range and
     *        the first (it starts in 0)
     *
     * @return The last evaluation point
     * @return std::nullopt on failure (e.g. curve not calculated)
     */
    virtual std::optional<double> getRange() const = 0;
    /**
     * @brief It computes the first derivative (orientation) at the given evaluation point.
     *
     * @return The derivative value
     * @return std::nullopt on wrong initialization
     */
    virtual std::optional<Orientation> evaluateOrientation(double evaluationPoint) = 0;
    /**
     * @brief It computes the derivative value at the given evaluation point. Currently only
     *        supports 1st and 2nd derivative (angular velocity and acceleration).
     *
     * @return The derivative value
     * @return std::nullopt on wrong initialization
     */
    virtual std::optional<Eigen::Vector3d> evaluate(double evaluationPoint,
        unsigned int derivative) = 0;
};

}  // namespace geometricmethods
}  // namespace math
}  // namespace crf
