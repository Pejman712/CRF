/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <vector>
#include <numeric>

#include "PeakDetection/CFAR.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace algorithms {
namespace peakdetection {

CFAR::CFAR(int numTrain, int numGuard, float falseAlarmRate):
    numTrain_(numTrain),
    numGuard_(numGuard),
    falseAlarmRate_(falseAlarmRate),
    logger_("CFAR") {
    logger_->debug("CTor");
    numTrainHalf_ = std::abs(numTrain_ / 2);
    numGuardHalf_ = std::abs(numGuard_ / 2);
    numSide_ = numTrainHalf_ + numGuardHalf_;
    alpha_ = static_cast<float>(numTrain_) *
        (pow(falseAlarmRate_, -1 / static_cast<float>(numTrain_)) -1);
}

CFAR::~CFAR() {
    logger_->debug("DTor");
}

std::vector<int> CFAR::findPeaks(const std::vector<float>& dataVec) {
    std::vector<int> peakMat;
    for (int i = numSide_; i < dataVec.size() - numSide_; i++) {
        float maxValPos = std::distance(dataVec.begin(), std::max_element(
            dataVec.begin()+i-numSide_, dataVec.begin()+i+numSide_));
        if (maxValPos != i) {
            continue;
        }
        float sumTotal = std::accumulate(dataVec.begin()+i-numSide_,
            dataVec.begin()+i+numSide_, 0.0);
        float sumGuard = std::accumulate(dataVec.begin()+i-numGuardHalf_,
            dataVec.begin()+i+numGuardHalf_, 0.0);
        float pNoise = (sumTotal - sumGuard) / numTrain_;
        if (dataVec[i] > alpha_ * pNoise) {
            peakMat.push_back(i);
        }
    }
    return peakMat;
}

}  // namespace peakdetection
}  // namespace algorithms
}  // namespace crf
