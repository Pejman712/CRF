/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#pragma once

namespace crf {
namespace math {
namespace inputshaper {

/**
 * @ingroup group_input_shaper
 * @brief Interface class for all the input shapers to be designed.
 *
 */

class IInputShaper {
 public:
    virtual ~IInputShaper() = default;

    /**
     * @ingroup group_input_shaper
     * @brief Method to reset the input shaper. All values will be erased and
     * the object will be fully reset.
     * @{
     */
    virtual void reset() = 0;
    /**
     * @brief Method to set the reference that the input shaper will aim to achieve
     *
     * @param reference The reference value
     *
     */
    virtual void setReference(const double& reference) = 0;
    /**
     * @brief Method to modify the responsiveness of the input shaper. The minimum value is 1
     * and can only be increased.
     *
     * @param factor The value of the responsiveness factor that will increase the response time
     * of the system.
     *
     */
    virtual void setResponsivenessFactor(const double& factor) = 0;
    /**
     * @brief Evaluates the input shaper in the time specified. Time can only move forward.
     *
     * @param evaluationPoint The time at which the shaper will be evaluated in seconds
     *
     * @return The value of the input shaper at the specified time
     */
    virtual double getInputPoint(const double& evaluationPoint) = 0;
};
/**@}*/

}  // namespace inputshaper
}  // namespace math
}  // namespace crf
