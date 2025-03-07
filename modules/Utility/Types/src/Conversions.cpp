/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Types/Conversions.hpp"

namespace crf::utility::types {

std::vector<double> stdVectorFromEigenVector(const Eigen::VectorXd& eigenVector) {
    size_t size = eigenVector.size();
    std::vector<double> stdVector(size);
    for (size_t i = 0; i < size; i++) {
        stdVector[i] = eigenVector(i);
    }
    return stdVector;
}

Eigen::VectorXd eigenVectorFromStdVector(const std::vector<double>& stdVector) {
    size_t size = stdVector.size();
    Eigen::VectorXd eigenVector(size);
    for (size_t i = 0; i < size; i++) {
        eigenVector(i) = stdVector[i];
    }
    return eigenVector;
}

template <size_t size>
std::array<double, size> stdArrayFromEigenVector(
    const Eigen::Vector<double, static_cast<int>(size)>& eigenVector) {
    std::array<double, size> stdArray;
    for (size_t i = 0; i < size; i++) {
        stdArray[i] = eigenVector(i);
    }
    return stdArray;
}

template std::array<double, 3> stdArrayFromEigenVector(
    const Eigen::Vector<double, 3>& eigenVector);
template std::array<double, 4> stdArrayFromEigenVector(
    const Eigen::Vector<double, 4>& eigenVector);
template std::array<double, 6> stdArrayFromEigenVector(
    const Eigen::Vector<double, 6>& eigenVector);

template <size_t size>
Eigen::Vector<double, static_cast<int>(size)> eigenVectorFromStdArray(
    const std::array<double, size>& stdArray) {
    Eigen::Vector<double, static_cast<int>(size)> eigenVector;
    for (size_t i = 0; i < size; i++) {
        eigenVector(i) = stdArray[i];
    }
    return eigenVector;
}

template Eigen::Vector<double, 3> eigenVectorFromStdArray(
    const std::array<double, 3>& stdArray);
template Eigen::Vector<double, 4> eigenVectorFromStdArray(
    const std::array<double, 4>& stdArray);
template Eigen::Vector<double, 6> eigenVectorFromStdArray(
    const std::array<double, 6>& stdArray);

}  // namespace crf::utility::types
