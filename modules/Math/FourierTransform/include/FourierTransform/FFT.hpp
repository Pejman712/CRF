/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <complex>

#include "EventLogger/EventLogger.hpp"

namespace crf::math::fouriertransform {

/**
 * @ingroup group_fourier_transform
 * @brief Class performs Discrete Fourier Transformation (DFT) using eigen FFT library
 * with added windowing and zero-padding features.
 * @{
 */
class FFT {
 public:
    FFT();
    ~FFT();

    /**
     * @brief Theory can be found in https://en.wikipedia.org/wiki/Discrete-time_Fourier_transform
     *        and https://en.wikipedia.org/wiki/Hann_function.
     * @param data: values
     * @param zeroPadding: represents zero-padded vector size, that is added in front and at the
     *        end of the original signal.
     * @param windowType: indicates if windowing (Hann) function should be applied to signal.
     * @return complex float vector representing frequency spectrum of the signal.
     * @return empty vector on failure.
     */
    std::vector<std::complex<float>> getFFT(std::vector<float> data, int zeroPadding = 0,
        bool windowType = false);

 private:
    crf::utility::logger::EventLogger logger_;
};
/**@}*/

}  // namespace crf::math::fouriertransform
