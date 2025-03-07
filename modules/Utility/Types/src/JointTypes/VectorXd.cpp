/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *         Bartosz Sójka BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include "Types/JointTypes/VectorXd.hpp"

namespace crf::utility::types {

VectorXd::VectorXd() :
    VectorXd(0) {
}

VectorXd::VectorXd(const std::size_t dimension) :
    coordinates_(Eigen::VectorXd::Zero(dimension)) {
    if (dimension < 0) {
        throw std::invalid_argument(
            "VectorXd(std::size_t): Provided dimension " + std::to_string(dimension) +
            " is negative.");
    }
}

VectorXd::VectorXd(const Eigen::VectorXd& coordinates) :
    coordinates_(coordinates) {
}

VectorXd::VectorXd(const std::vector<double>& coordinates) :
    coordinates_(Eigen::VectorXd::Zero(coordinates.size())) {
    for (std::size_t i = 0; i < coordinates.size(); i++) {
        coordinates_(i) = coordinates[i];
    }
}

VectorXd::VectorXd(const std::initializer_list<double>& coordinates) :
    VectorXd(std::vector<double>(coordinates)) {
}

VectorXd& VectorXd::operator=(const Eigen::VectorXd& coordinates) {
    coordinates_ = coordinates;
    return *this;
}

VectorXd& VectorXd::operator=(const std::vector<double>& coordinates) {
    for (std::size_t i = 0; i < size(); i++) {
        coordinates_(i) = coordinates[i];
    }
    return *this;
}

VectorXd& VectorXd::operator=(const std::initializer_list<double>& coordinates) {
    *this = std::vector<double>(coordinates);
    return *this;
}

std::size_t VectorXd::size() const {
    return static_cast<std::size_t>(coordinates_.size());
}

Eigen::VectorXd VectorXd::raw() const {
    return coordinates_;
}

double& VectorXd::operator[](const std::size_t index) {
    if (index < 0) {
        throw std::out_of_range(
            "VectorXd::operator[]: Index " + std::to_string(index) + "is out of range [0, " +
            std::to_string(size()) + "].");
    }
    if (index < size()) {
        return coordinates_(index);
    }
    throw std::out_of_range(
        "VectorXd::operator[]: Index " + std::to_string(index) + "is out of range [0, " +
        std::to_string(size()) + "].");
}

double VectorXd::operator[](const std::size_t index) const {
    if (index < 0) {
        throw std::out_of_range(
            "VectorXd::operator[] const: Index " + std::to_string(index) + "is out of range [0, " +
            std::to_string(size()) + "].");
    }
    if (index < size()) {
        return coordinates_(index);
    }
    throw std::out_of_range(
        "VectorXd::operator[] const: Index " + std::to_string(index) + "is out of range [0, " +
        std::to_string(size()) + "].");
}

std::ostream& operator<<(std::ostream& os, const VectorXd& vectorXd) {
    os << std::fixed << std::setprecision(15);
    os << vectorXd.raw();
    return os;
}

}  // namespace crf::utility::types
