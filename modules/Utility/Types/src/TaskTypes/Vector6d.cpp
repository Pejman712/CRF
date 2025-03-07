/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *         Zsolt Pasztori CERN EN/SMM/MRO
 *         Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *         Bartosz Sójka BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include "Types/TaskTypes/Vector6d.hpp"

namespace crf::utility::types {

Vector6d::Vector6d() :
    coordinates_(Eigen::Vector<double, 6>::Zero()) {
}

Vector6d::Vector6d(const Eigen::Vector<double, 6>& coordinates) :
    coordinates_(coordinates) {
}

Vector6d::Vector6d(const std::array<double, 6>& coordinates) :
    coordinates_(Eigen::Vector<double, 6>::Zero()) {
    for (size_t i = 0; i < size(); i++) {
        coordinates_(i) = coordinates[i];
    }
}

Vector6d::Vector6d(const std::initializer_list<double>& coordinates) {
    if (coordinates.size() != size()) {
        throw std::invalid_argument(
            "Vector6d(std::initializer_list<double>): VectorXd object can only be "
            "constructed from initialiser list of 6 elements, but coordinates "
            "argument has size " +
            std::to_string(coordinates.size()) + ".");
    } else {
        std::vector<double> stdVectorCoordinates(coordinates);
        for (std::size_t i = 0; i < size(); i++) {
            coordinates_(i) = stdVectorCoordinates[i];
        }
    }
}

Vector6d& Vector6d::operator=(const Eigen::Vector<double, 6>& coordinates) {
    coordinates_ = coordinates;
    return *this;
}

Vector6d& Vector6d::operator=(const std::array<double, 6>& coordinates) {
    for (std::size_t i = 0; i < size(); i++) {
        coordinates_(i) = coordinates[i];
    }
    return *this;
}

Vector6d& Vector6d::operator=(const std::initializer_list<double>& coordinates) {
    if (coordinates.size() != size()) {
        throw std::invalid_argument(
            "Vector6d operator=(std::initializer_list<double>): "
            "Vector6d object has size " +
            std::to_string(size()) + ", but std::initializer_list<double> argument has size " +
            std::to_string(coordinates.size()) + ".");
    } else {
        std::vector<double> stdVectorCoordinates(coordinates);
        for (std::size_t i = 0; i < size(); i++) {
            coordinates_(i) = stdVectorCoordinates[i];
        }
    }
    return *this;
}

std::size_t Vector6d::size() const {
    return static_cast<std::size_t>(6);
}

Eigen::Vector<double, 6> Vector6d::raw() const {
    return coordinates_;
}

double& Vector6d::operator[](const std::size_t index) {
    if (index < 0) {
        throw std::out_of_range(
            "Vector6d::operator[]: Index " + std::to_string(index) + "is out of range [0, " +
            std::to_string(size()) + "].");
    }
    if (index < size()) {
        return coordinates_(index);
    }
    throw std::out_of_range(
        "Vector6d::operator[]: Index " + std::to_string(index) + "is out of range [0, " +
        std::to_string(size()) + "].");
}

double Vector6d::operator[](const std::size_t index) const {
    if (index < 0) {
        throw std::out_of_range(
            "Vector6d::operator[] const: Index " + std::to_string(index) + "is out of range [0, " +
            std::to_string(size()) + "].");
    }
    if (index < size()) {
        return coordinates_(index);
    }
    throw std::out_of_range(
        "Vector6d::operator[] const: Index " + std::to_string(index) + "is out of range [0, " +
        std::to_string(size()) + "].");
}

std::ostream& operator<<(std::ostream& os, const Vector6d& vector6d) {
    os << std::fixed << std::setprecision(15);
    os << vector6d.raw();
    return os;
}

}  // namespace crf::utility::types
