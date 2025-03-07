#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>

namespace crf {
namespace algorithms {
namespace peakdetection {

/**
 * @ingroup group_peak_detection
 * Interface class for PeakDetection methods.
 * Known subclasses: CFAR, GradientPeakDetection.
 * @{
*/

class IPeakDetection {
 public:
    virtual ~IPeakDetection() = default;
    /**
     * Returns vector of ints, that represent index of peak values in the provided data vec.
    */
    virtual std::vector<int> findPeaks(const std::vector<float>& dataVec) = 0;
};
/**@}*/

}  // namespace peakdetection
}  // namespace algorithms
}  // namespace crf
