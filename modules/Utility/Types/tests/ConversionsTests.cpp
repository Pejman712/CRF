/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::Return;

using crf::utility::types::stdVectorFromEigenVector;
using crf::utility::types::eigenVectorFromStdVector;
using crf::utility::types::stdArrayFromEigenVector;
using crf::utility::types::eigenVectorFromStdArray;

class ConversionsShould : public ::testing::Test {
 protected:
    ConversionsShould() :
        stdVector7_(
            {-1.2452654789023454,
             3.1487283479344533,
             2.8456356344354765,
             4.0,
             23.134142312313420,
             0.0001231242,
             5.22784}),
        eigenVector7_(7),
        stdArray3_({1.1471123464639490, -1.6415867259093058, 0.1139727950424842}),
        stdArray4_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        stdArray6_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}),
        eigenVector3_({1.1471123464639490, -1.6415867259093058, 0.1139727950424842}),
        eigenVector4_(
            {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799}),
        eigenVector6_({2.2, -3.8, 4.7, 5.8, 4.7, 3.9}) {
        eigenVector7_ << -1.2452654789023454, 3.1487283479344533, 2.8456356344354765, 4.0,
            23.134142312313420, 0.0001231242, 5.22784;
    }

 protected:
    const std::array<double, 3> stdArray3_;
    const std::array<double, 4> stdArray4_;
    const std::array<double, 6> stdArray6_;
    const Eigen::Vector<double, 3> eigenVector3_;
    const Eigen::Vector<double, 4> eigenVector4_;
    const Eigen::Vector<double, 6> eigenVector6_;

    const std::vector<double> stdVector7_;
    Eigen::VectorXd eigenVector7_;
};

TEST_F(ConversionsShould, CorrectlyConvertBetweenStdVectorAndEigenVector) {
    ASSERT_EQ(stdVector7_, stdVectorFromEigenVector(eigenVector7_));
    ASSERT_EQ(eigenVector7_, eigenVectorFromStdVector(stdVector7_));
}

TEST_F(ConversionsShould, CorrectlyConvertBetweenStdArrayAndEigenVector) {
    ASSERT_EQ(stdArray3_, stdArrayFromEigenVector<3>(eigenVector3_));
    ASSERT_EQ(eigenVector3_, eigenVectorFromStdArray<3>(stdArray3_));

    ASSERT_EQ(stdArray4_, stdArrayFromEigenVector<4>(eigenVector4_));
    ASSERT_EQ(eigenVector4_, eigenVectorFromStdArray<4>(stdArray4_));

    ASSERT_EQ(stdArray6_, stdArrayFromEigenVector<6>(eigenVector6_));
    ASSERT_EQ(eigenVector6_, eigenVectorFromStdArray<6>(stdArray6_));
}
