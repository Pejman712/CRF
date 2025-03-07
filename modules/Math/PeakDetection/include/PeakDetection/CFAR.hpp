#pragma once

/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>

#include "PeakDetection/IPeakDetection.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace algorithms {
namespace peakdetection {
/**
 * @ingroup group_peak_detection
 * @{
*/
class CFAR : public IPeakDetection {
 public:
    CFAR(int numTrain, int numGuard, float falseAlarmRate);
    CFAR() = delete;
    ~CFAR();
    std::vector<int> findPeaks(const std::vector<float>& dataVec);

 private:
    utility::logger::EventLogger logger_;
    float falseAlarmRate_, alpha_;
    int numTrain_, numGuard_, numTrainHalf_, numGuardHalf_, numSide_;
};
/**@}*/

}  // namespace peakdetection
}  // namespace algorithms
}  // namespace crf
