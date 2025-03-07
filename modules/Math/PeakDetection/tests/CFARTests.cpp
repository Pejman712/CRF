/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "PeakDetection/CFAR.hpp"

using testing::_;

class PeakDetectionShould: public ::testing::Test {
 protected:
    PeakDetectionShould(): logger_("PeakDetectionShould") {
        logger_->info("{0} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }

    ~PeakDetectionShould() {
        logger_->info("{0} END with {1}",
        testing::UnitTest::GetInstance()->current_test_info()->name(),
        testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    crf::utility::logger::EventLogger logger_;
    std::unique_ptr<crf::algorithms::peakdetection::CFAR> sut_;
};

TEST_F(PeakDetectionShould, detectCorrectPeaksAndCorrectPeakAmount) {
    std::vector<float> fakeDataVector;
    float randomNumber;
    for (size_t j = 0; j < 1000; j++) {
        randomNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX); // NOLINT
        fakeDataVector.push_back(randomNumber);
    }
    fakeDataVector[356] = 5;
    fakeDataVector[758] = 5;
    sut_.reset(new crf::algorithms::peakdetection::CFAR(10, 40, 0.01));
    std::vector<int> peakVec = sut_->findPeaks(fakeDataVector);
    ASSERT_EQ(peakVec.size(), 2);
    ASSERT_EQ(peakVec[0], 356);
    ASSERT_EQ(peakVec[1], 758);
}
