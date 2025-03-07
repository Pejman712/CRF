/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "FourierTransform/FFT.hpp"

using testing::_;

class FFTShould: public ::testing::Test {
 protected:
    FFTShould(): logger_("FFTShould") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~FFTShould() {
        logger_->info("{0} END with {1}",
        testing::UnitTest::GetInstance()->current_test_info()->name(),
        testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::math::fouriertransform::FFT> sut_;
};

// The  FFT Results were compared manually calculating DFT for each simple case.

TEST_F(FFTShould, returnEmptyVectorIfReceivedEmptyDataVec) {
    std::vector<float> mockData;
    sut_.reset(new crf::math::fouriertransform::FFT());
    std::vector<std::complex<float>> result = sut_->getFFT(mockData);
    ASSERT_TRUE(result.empty());
}

TEST_F(FFTShould, getOnlyDcComponentIfNoWindowOrPaddingApplied) {
    std::vector<float> mockData(4, 1);
    sut_.reset(new crf::math::fouriertransform::FFT());
    std::vector<std::complex<float>> result = sut_->getFFT(mockData);
    ASSERT_NEAR(real(result[0]), 4, 0.01);
    ASSERT_NEAR(real(result[1]), 0, 0.01);
}

TEST_F(FFTShould, assertZeroPaddingIsNotNegative) {
    std::vector<float> mockData(4, 1);
    sut_.reset(new crf::math::fouriertransform::FFT());
    std::vector<std::complex<float>> result = sut_->getFFT(mockData, -2);
    ASSERT_TRUE(result.empty());
}

TEST_F(FFTShould, assertCorrectFrequencyAmplitudesWhenPaddingUsed) {
    std::vector<float> mockData(4, 1);
    sut_.reset(new crf::math::fouriertransform::FFT());
    std::vector<std::complex<float>> result = sut_->getFFT(mockData, 2);
    ASSERT_NEAR(real(result[1]), -2.41, 0.01);
    ASSERT_NEAR(real(result[2]), 0, 0.01);
}

TEST_F(FFTShould, getComponent) {
    std::vector<float> mockData(4, 1);
    sut_.reset(new crf::math::fouriertransform::FFT());
    std::vector<std::complex<float>> result = sut_->getFFT(mockData, 2, true);
    ASSERT_NEAR(real(result[1]), -1.7, 0.01);
    ASSERT_NEAR(real(result[2]), 1, 0.01);
}
