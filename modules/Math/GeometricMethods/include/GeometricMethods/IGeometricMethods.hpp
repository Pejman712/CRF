/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
 */

#pragma once

#include <optional>

namespace crf {
namespace math {
namespace geometricmethods {

/**
 * @ingroup group_geometric_methods
 * @brief
*/
class IGeometricMethods {
 public:
    virtual ~IGeometricMethods() = default;
    /**
     * @brief It calculates the difference between the last evaluation point of the cure range and
     *        the first (it starts in 0)
     * 
     * @return The last evaluation point
     * @return std::nullopt on failure (e.g. curve not calculated)
     */
    virtual std::optional<double> getRange() const = 0;
    /**
     * @brief It computes the derivative value at the given evaluation point for de correspondent
     *        derivative
     * 
     * @return The derivative value
     * @return std::nullopt on wrong initialization
     */
    virtual std::optional<double> evaluate(double evaluationPoint, unsigned int derivative) = 0;
};

}  // namespace geometricmethods
}  // namespace math
}  // namespace crf
