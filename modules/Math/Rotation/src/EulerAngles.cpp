/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include "Rotation/EulerAngles.hpp"

namespace crf::math::rotation {

EulerAngles::EulerAngles() :
    angles_({0.0, 0.0, 0.0}) {
}

EulerAngles::EulerAngles(const std::array<double, 3>& angles) :
    angles_(angles) {
}

EulerAngles::EulerAngles(const std::initializer_list<double>& angles) {
    if (angles.size() != 3) {
        throw std::invalid_argument(
            "EulerAngles can only be constructed from initialiser list of 3 elements.");
    } else {
        std::vector<double> stdVectorAngles(angles);
        for (std::size_t i = 0; i < size(); i++) {
            angles_[i] = stdVectorAngles[i];
        }
    }
}

EulerAngles& EulerAngles::operator=(const std::array<double, 3>& angles) {
    angles_ = angles;
    return *this;
}

EulerAngles& EulerAngles::operator=(const std::initializer_list<double>& angles) {
    if (angles.size() != 3) {
        throw std::invalid_argument(
            "EulerAngles can only be constructed from initialiser list of 3 elements.");
    } else {
        std::vector<double> stdVectorAngles(angles);
        for (std::size_t i = 0; i < size(); i++) {
            angles_[i] = stdVectorAngles[i];
        }
    }
    return *this;
}

double& EulerAngles::operator[](const std::size_t index) {
    if (index < 0) {
        throw std::out_of_range("EulerAngles::operator[]: Out of range index");
    }
    if (index < 3) {
        return angles_[index];
    }
    throw std::out_of_range("EulerAngles::operator[]: Out of range index");
}

double EulerAngles::operator[](const std::size_t index) const {
    if (index < 0) {
        throw std::out_of_range(
            "EulerAngles::operator[]: Index " + std::to_string(index) + "is out of range [0, " +
            std::to_string(size()) + "].");
    }
    if (index < 3) {
        return angles_[index];
    }
    throw std::out_of_range(
        "EulerAngles::operator[]: Index " + std::to_string(index) + "is out of range [0, " +
        std::to_string(size()) + "].");
}

std::size_t EulerAngles::size() const {
    return static_cast<std::size_t>(3);
}

std::array<double, 3> EulerAngles::rawArray() const {
    return angles_;
}

std::ostream& operator<<(std::ostream& os, const EulerAngles& eulerAngles) {
    os << std::fixed << std::setprecision(15);
    std::array<std::size_t, 3> stringsLengths({1, 1, 1});
    std::size_t maxStringLength = 0;
    for (std::size_t i = 0; i < eulerAngles.size(); i++) {
        unsigned int quotient = static_cast<unsigned int>(abs(eulerAngles[i]));
        while (quotient / 10 > 0) {
            stringsLengths[i]++;
            quotient /= 10;
        }
        if (eulerAngles[i] < 0) {
            stringsLengths[i]++;
        }
        if (stringsLengths[i] > maxStringLength) {
            maxStringLength = stringsLengths[i];
        }
    }
    for (std::size_t i = 0; i < eulerAngles.size(); i++) {
        if (i > 0) {
            os << "\n";
        }
        for (std::size_t j = 0; j < maxStringLength - stringsLengths[i]; j++) {
            os << " ";
        }
        os << eulerAngles[i];
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const CardanXYZ& cardanXYZ) {
    std::array<std::string, 3> axes({"X", "Y", "Z"});
    os << std::fixed << std::setprecision(15);
    std::array<std::size_t, 3> stringsLengths({1, 1, 1});
    std::size_t maxStringLength = 0;
    for (std::size_t i = 0; i < cardanXYZ.size(); i++) {
        unsigned int quotient = static_cast<unsigned int>(abs(cardanXYZ[i]));
        while (quotient / 10 > 0) {
            stringsLengths[i]++;
            quotient /= 10;
        }
        if (cardanXYZ[i] < 0) {
            stringsLengths[i]++;
        }
        if (stringsLengths[i] > maxStringLength) {
            maxStringLength = stringsLengths[i];
        }
    }
    for (std::size_t i = 0; i < cardanXYZ.size(); i++) {
        if (i > 0) {
            os << "\n";
        }
        os << axes[i] << ": ";
        for (std::size_t j = 0; j < maxStringLength - stringsLengths[i]; j++) {
            os << " ";
        }
        os << cardanXYZ[i];
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const EulerZXZ& eulerZXZ) {
    std::array<std::string, 3> axes({"Z", "X", "Z"});
    os << std::fixed << std::setprecision(15);
    std::array<std::size_t, 3> stringsLengths({1, 1, 1});
    std::size_t maxStringLength = 0;
    for (std::size_t i = 0; i < eulerZXZ.size(); i++) {
        unsigned int quotient = static_cast<unsigned int>(abs(eulerZXZ[i]));
        while (quotient / 10 > 0) {
            stringsLengths[i]++;
            quotient /= 10;
        }
        if (eulerZXZ[i] < 0) {
            stringsLengths[i]++;
        }
        if (stringsLengths[i] > maxStringLength) {
            maxStringLength = stringsLengths[i];
        }
    }
    for (std::size_t i = 0; i < eulerZXZ.size(); i++) {
        if (i > 0) {
            os << "\n";
        }
        os << axes[i] << ": ";
        for (std::size_t j = 0; j < maxStringLength - stringsLengths[i]; j++) {
            os << " ";
        }
        os << eulerZXZ[i];
    }
    return os;
}

}  // namespace crf::math::rotation
