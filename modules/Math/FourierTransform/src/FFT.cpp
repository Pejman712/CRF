/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <math.h>
#include <vector>

#include <eigen3/unsupported/Eigen/FFT>

#include "FourierTransform/FFT.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::math::fouriertransform {

FFT::FFT():
  logger_("FFT") {
    logger_->debug("CTor");
}

FFT::~FFT() {
    logger_->debug("DTor");
}

std::vector<std::complex<float>> FFT::getFFT(std::vector<float> data, int zeroPadding,
    bool window) {
    logger_->debug("getFFT");
    std::vector<std::complex<float>> fftData;
    if (data.empty()) {
        logger_->warn("received empty data!");
        return fftData;
    }
    if (window) {
        for (std::size_t j = 0; j < data.size(); j++) {
            double multiplier = 0.5 * (1 - cos(2*M_PI*j/data.size()));
            data[j] = multiplier * data[j];
        }
    }
    if (zeroPadding != 0) {
        if (zeroPadding < 0) {
            logger_->error("Negative zero padding value received!");
            return fftData;
        }
        std::vector<float> zerosVector(zeroPadding, 0);
        data.insert(data.end(), zerosVector.begin(), zerosVector.end());
        data.insert(data.begin(), zerosVector.begin(), zerosVector.end());
    }
    Eigen::FFT<float> fft;
    fft.fwd(fftData, data);
    return fftData;
}

}  // namespace crf::math::fouriertransform
