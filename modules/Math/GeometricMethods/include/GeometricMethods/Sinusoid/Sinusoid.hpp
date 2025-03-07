/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2021
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
 */

#pragma once

#include <optional>

#include "EventLogger/EventLogger.hpp"
#include "GeometricMethods/IGeometricMethods.hpp"

namespace crf {
namespace math {
namespace geometricmethods {

enum class ComputationMethod {
    /**
     * @ingroup group_sinusoid
     * @brief The sinusoid curve is generated according to the maximun first derivative
     */
    Limit1stDerivative = 0,
    /**
     * @brief The sinusoid curve is generated according to a fixed range,
     *        with e.g. definitionValue as upper boundary range = [0, definitionValue]
     */
    SetRange = 1
};

/**
 * @brief Generates a generic sinusoid trajectory. The 0th derivative should be considered as the
 *        main output. With start0thDerivative and end0thDerivative the start and end value of the
 *        trajectory can be set. There are 2 ways to define/set the trajectory (using :
 *            1. Limit 1st Derivative of Trajectory:
 *               The 1st derivate can be limited with ComputationMethod::Limit1stDerivative and
 *               definitionValue.
 *               E.G.:
 *               Planning a trajectory for velocity control: Use 0th derivative/main output as
 *               velocity.
 *               Set start and end velocity with start0thDerivative and end0thDerivative. Limit
 *               acceleration (1st derivative) with definitionValue.
 *            2. Set the Range of the Trajectory:
 *               The range can be set with ComputationMethod::setRange and definitionValue. Thus
 *               the trajectory will be defined over range = [0, definitionValue].
 *        Using 0th derivative as velocity results in a C4 trajectory.
 *        (0th derivative <-> position => C3, 0th derivative <-> velocity => C4, 
 *        0th derivative <-> acceleration => C5, etc. ...)
 * 
 * @param start0thDerivative is the initial value of 0 derivative
 * @param end0thDerivative is the objective (final) value of 0 derivative
 * @param definitionValue corresponds to the limit value of the first derivative or to the selected
 *        range according to the chosen method
 * @param method selected can be Limit1stDerivative or SetRange
 */
class Sinusoid: public IGeometricMethods {
 public:
    Sinusoid(double start0thDerivative, double end0thDerivative, double definitionValue,
      ComputationMethod method = crf::math::geometricmethods::ComputationMethod::Limit1stDerivative);  // NOLINT
    ~Sinusoid() override;

    std::optional<double> getRange() const override;
    std::optional<double> evaluate(double evaluationPoint, unsigned int derivative) override;

 private:
    double start0thDerivative_;
    crf::utility::logger::EventLogger logger_;
    double curveScalingFactor_;
    double max2ndDerivative_;
    double w_;
};

}  // namespace geometricmethods
}  // namespace math
}  // namespace crf
