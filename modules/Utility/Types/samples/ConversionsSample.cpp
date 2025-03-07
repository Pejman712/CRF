/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Types/Conversions.hpp"

using crf::utility::types::stdVectorFromEigenVector;
using crf::utility::types::eigenVectorFromStdVector;
using crf::utility::types::stdArrayFromEigenVector;
using crf::utility::types::eigenVectorFromStdArray;

int main() {
    /**
     * Sample variables for conversion
    */
    std::vector<double> stdVector7(
        {-1.2452654789023454,
         3.1487283479344533,
         2.8456356344354765,
         4.0,
         23.134142312313420,
         0.0001231242,
         5.22784});
    Eigen::VectorXd eigenVector7(7);
    eigenVector7 << -1.2452654789023454, 3.1487283479344533, 2.8456356344354765, 4.0,
        23.134142312313420, 0.0001231242, 5.22784;
    std::array<double, 3> stdArray3({1.1471123464639490, -1.6415867259093058, 0.1139727950424842});
    std::array<double, 4> stdArray4(
        {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799});
    std::array<double, 6> stdArray6({2.2, -3.8, 4.7, 5.8, 4.7, 3.9});
    Eigen::Vector<double, 3> eigenVector3(
        {1.1471123464639490, -1.6415867259093058, 0.1139727950424842});
    Eigen::Vector<double, 4> eigenVector4(
        {0.5505666516902112, -0.6362152372281484, 0.3613804315900029, 0.4018839604029799});
    Eigen::Vector<double, 6> eigenVector6({2.2, -3.8, 4.7, 5.8, 4.7, 3.9});

    /**
     * Functions enable conversions back and forth between std::vector<double> and Eigen::VectorXd,
     * and between std::array<double, size> and Eigen::Vector<double, size>, where
     * size equals to 3, 4 or 6.
    */
    std::vector<double> stdVector = stdVectorFromEigenVector(eigenVector7);
    Eigen::VectorXd eigenVector = eigenVectorFromStdVector(stdVector7);

    std::array<double, 3> stdArray3Output = stdArrayFromEigenVector<3>(eigenVector3);
    Eigen::Vector3d eigenVector3output = eigenVectorFromStdArray<3>(stdArray3);

    std::array<double, 4> stdArray4Output = stdArrayFromEigenVector<4>(eigenVector4);
    Eigen::Vector4d eigenVector4Output = eigenVectorFromStdArray<4>(stdArray4);

    std::array<double, 6> stdArray6Output = stdArrayFromEigenVector<6>(eigenVector6);
    Eigen::Vector<double, 6> eigenVector6Output = eigenVectorFromStdArray<6>(stdArray6);

    return 0;
}
