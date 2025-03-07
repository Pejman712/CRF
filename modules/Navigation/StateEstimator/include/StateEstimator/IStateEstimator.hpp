#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <vector>

namespace crf {
namespace algorithms {
namespace stateestimator {

enum StateEstimatorFilterType {
    UNDEFINED_KF = 0,
    UNSCENTED_KF = 1,
    EXTENDED_KF = 2
};

class IStateEstimator {
 public:
    virtual ~IStateEstimator() = default;
     /*
     * Returns floating point array of the currently state estimation
     * Returns empty vector upon failure
     */
    virtual std::vector<float> getEstimate() = 0;
     /* 
     * Returns:
     *  - True if measured values have been successfully added to estimation algorithm
     *  - False upon failure
     */
    virtual bool addMeasurement(const std::vector<float>& measuredValues) = 0;
};

}  // namespace stateestimator
}  // namespace algorithms
}  // namespace crf
