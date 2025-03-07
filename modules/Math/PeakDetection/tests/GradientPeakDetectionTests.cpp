/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "PeakDetection/GradientPeakDetection.hpp"

using testing::_;

class GradientPeakDetectionShould: public ::testing::Test {
 protected:
    GradientPeakDetectionShould(): logger_("GradientPeakDetectionShould") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~GradientPeakDetectionShould() {
        logger_->info("{0} END with {1}",
        testing::UnitTest::GetInstance()->current_test_info()->name(),
        testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::algorithms::peakdetection::GradientPeakDetection> sut_;
};

TEST_F(GradientPeakDetectionShould, DetectAllPositivePeaksOfSinusoidWave) {
    std::vector<float> y;
    for (int i = 0; i < 1000; i++) {
        y.push_back(sin(M_PI * 0.01 * i));
    }
    sut_.reset(new crf::algorithms::peakdetection::GradientPeakDetection());
    std::vector<int> peakVec = sut_->findPeaks(y);
    ASSERT_EQ(peakVec[0], 50);
    ASSERT_EQ(peakVec[1], 250);
}
