/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>

#include "PeakDetection/GradientPeakDetection.hpp"
#include "EventLogger/EventLogger.hpp"

#define EPS 1e-16

namespace crf {
namespace algorithms {
namespace peakdetection {

GradientPeakDetection::GradientPeakDetection(float thresholdValue):
  thresholdValue_(thresholdValue),
  logger_("GradientPeakDetection") {
}

GradientPeakDetection::~GradientPeakDetection() {
}

std::vector<int> GradientPeakDetection::findPeaks(const std::vector<float>& dataVec) {
    std::vector<float> differenceVec = getDifference(dataVec);
    // Makes sure there is no absolute 0
    std::replace(differenceVec.begin(), differenceVec.end(), 0.0, -EPS);
    std::vector<float> gradientVec = getGradient(differenceVec);
    std::vector<int> peakIndVec = getPeakLocations(gradientVec);
    std::vector<float> peakAmplitudeVec = getPeakAmplitude(peakIndVec, dataVec);
    std::vector<int> posPeakIndVec = getPositivePeaks(peakAmplitudeVec, peakIndVec);
    std::vector<int> aboveThresholdPeakIndVec = getPeaksAboveThreshold(
    dataVec, posPeakIndVec);
    return aboveThresholdPeakIndVec;
}

std::vector<float> GradientPeakDetection::getDifference(const std::vector<float>& dataVec) {
    std::vector<float> differenceVec;
    for (int i = 1; i < dataVec.size(); ++i) {
        differenceVec.push_back(dataVec[i] - dataVec[i-1]);
    }
  return differenceVec;
}

std::vector<float> GradientPeakDetection::getGradient(const std::vector<float>& differenceVec) {
    std::vector<float> gradientVec;
    for (int i = 1; i < differenceVec.size(); ++i) {
        gradientVec.push_back(differenceVec[i] * differenceVec[i-1]);
    }
  return gradientVec;
}

std::vector<int> GradientPeakDetection::getPeakLocations(const std::vector<float>& gradientVec) {
    std::vector<int> indexVec;
    for (int i = 0; i < gradientVec.size(); ++i) {
        if (gradientVec[i] < 0) {
            indexVec.push_back(i+1);
        }
    }
    return indexVec;
}

std::vector<float> GradientPeakDetection::getPeakAmplitude(const std::vector<int>& indexVec,
    const std::vector<float>& dataVec) {
    std::vector<float> peakAmplitudeVec;
    for (int i = 0; i < indexVec.size(); ++i) {
        peakAmplitudeVec.push_back(dataVec[indexVec[i]]);
    }
    return peakAmplitudeVec;
}

std::vector<int> GradientPeakDetection::getPositivePeaks(std::vector<float> peakAmplitudeVec,
    std::vector<int> peakIndVec) {
    std::vector<int> posPeakIndVec;
    for (int i = 1; i < peakAmplitudeVec.size(); ++i) {
        if (peakAmplitudeVec[i] - peakAmplitudeVec[i-1] < 0) {
            posPeakIndVec.push_back(peakIndVec[i-1]);
        }
    }
    return posPeakIndVec;
}

std::vector<int> GradientPeakDetection::getPeaksAboveThreshold(const std::vector<float>& dataVec,
    const std::vector<int>& posPeakIndVec) {
    std::vector<int> aboveThresholdPeakIndVec;
    for (int i = 0; i < posPeakIndVec.size(); ++i) {
        if (dataVec[posPeakIndVec[i]] >= thresholdValue_) {
            aboveThresholdPeakIndVec.push_back(posPeakIndVec[i]);
        }
    }
    return aboveThresholdPeakIndVec;
}

}  // namespace peakdetection
}  // namespace algorithms
}  // namespace crf
