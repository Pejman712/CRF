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
class GradientPeakDetection : public IPeakDetection {
 public:
    explicit GradientPeakDetection(float thresholdValue = 0.01);
    ~GradientPeakDetection();
    std::vector<int> findPeaks(const std::vector<float>& dataVec);

 private:
    std::vector<float> getDifference(const std::vector<float>& signalVec);
    std::vector<float> getGradient(const std::vector<float>& differenceVec);
    std::vector<int> getPeakLocations(const std::vector<float>& gradientVec);
    std::vector<float> getPeakAmplitude(const std::vector<int>& indexVec,
        const std::vector<float>& signalVec);
    std::vector<int> getPositivePeaks(std::vector<float> peakAmplitudeVec,
        std::vector<int> peakIndVec);
    std::vector<int> getPeaksAboveThreshold(const std::vector<float>& signalVec,
        const std::vector<int>& posPeakIndVec);

    utility::logger::EventLogger logger_;
    float thresholdValue_;
};
/**@}*/

}  // namespace peakdetection
}  // namespace algorithms
}  // namespace crf
