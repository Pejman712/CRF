/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include "Types/Comparison.hpp"

namespace crf::utility::types {

bool areAlmostEqual(const VectorXd& vector1, const VectorXd& vector2, const double& accuracy) {
    if (vector1.size() != vector2.size()) {
        throw std::invalid_argument(
            "crf::utility::types::areAlmostEqual: Function areAlmostEqual can only "
            "take arguments with the same number of elements.");
    }
    for (size_t i = 0; i < vector2.size(); i++) {
        if (!(abs(vector1[i] - vector2[i]) < accuracy)) {
            if (!((vector1[i] == std::numeric_limits<double>::infinity() &&
                   vector2[i] == std::numeric_limits<double>::infinity()) ||
                  (vector1[i] == -std::numeric_limits<double>::infinity() &&
                   vector2[i] == -std::numeric_limits<double>::infinity()))) {
                return false;
            }
        }
    }
    return true;
}

bool areAlmostEqual(const TaskPose& taskPose1, const TaskPose& taskPose2, const double& accuracy,
    const TaskSpace& mask) {
    Eigen::Vector3d position1 = taskPose1.getPosition();
    Eigen::Vector3d position2 = taskPose2.getPosition();
    for (size_t i = 0; i < 3; i++) {
        if (mask[i] && !(abs(position1(i) - position2(i)) < accuracy)) {
            return false;
        }
    }
    if (mask.angularDimension() == 0) {
        return true;
    }
    if (mask.angularDimension() == 1) {
        CardanXYZ cardanXYZ1 = taskPose1.getCardanXYZ();
        CardanXYZ cardanXYZ2 = taskPose2.getCardanXYZ();
        if (mask[TaskSpaceTangentDimension::Wx]) {
            return abs(cardanXYZ1[0] - cardanXYZ2[0]) < accuracy;
        }
        if (mask[TaskSpaceTangentDimension::Wy]) {
            return abs(cardanXYZ1[1] - cardanXYZ2[1]) < accuracy;
        }
        if (mask[TaskSpaceTangentDimension::Wz]) {
            return abs(cardanXYZ1[2] - cardanXYZ2[2]) < accuracy;
        }
        throw std::logic_error("areAlmostEqual(): mask.angularDimension() was equal to 1, but no "
            "angular dimensions are enabled.");
    }
    return areAlmostEqual(taskPose1.getOrientation(), taskPose2.getOrientation(), accuracy);
}

bool areAlmostEqual(const Vector6d& vector1, const Vector6d& vector2, const double& accuracy,
    const TaskSpace& mask) {
    for (size_t i = 0; i < 6; i++) {
        if (mask[i] && !(abs(vector1[i] - vector2[i]) < accuracy)) {
            if (!((vector1[i] == std::numeric_limits<double>::infinity() &&
                   vector2[i] == std::numeric_limits<double>::infinity()) ||
                  (vector1[i] == -std::numeric_limits<double>::infinity() &&
                   vector2[i] == -std::numeric_limits<double>::infinity()))) {
                return false;
            }
        }
    }
    return true;
}

bool areEqual(const TaskSpace& taskSpace1, const TaskSpace& taskSpace2) {
    for (size_t i = 0; i < 6; i++) {
        if (taskSpace1[i] != taskSpace2[i]) {
            return false;
        }
    }
    return true;
}

bool isBetween(const VectorXd& lowerBound, const VectorXd& upperBound, const VectorXd& vector) {
    if (lowerBound.size() != vector.size() || vector.size() != upperBound.size()) {
        throw std::invalid_argument("Function isBetween can only take arguments with the same "
            "number of elements.");
    }
    for (size_t i = 0; i < vector.size(); i++) {
        if (!(lowerBound[i] <= vector[i] && vector[i] <= upperBound[i])) {
            return false;
        }
    }
    return true;
}

bool isBetween(const Vector6d& lowerBound, const Vector6d& upperBound, const Vector6d& vector,
    const TaskSpace& mask) {
    for (size_t i = 0; i < 6; i++) {
        if (mask[i] && !(lowerBound[i] <= vector[i] && vector[i] <= upperBound[i])) {
            return false;
        }
    }
    return true;
}

bool isLesser(const VectorXd& vector1, const VectorXd& vector2) {
    if (vector1.size() != vector2.size()) {
        throw std::invalid_argument("Function isBetween can only take arguments with the same "
            "number of elements.");
    }
    for (size_t i = 0; i < vector2.size(); i++) {
        if (!(vector1[i] <= vector2[i])) {
            return false;
        }
    }
    return true;
}

bool isLesser(const Vector6d& vector1, const Vector6d& vector2, const TaskSpace& mask) {
    for (size_t i = 0; i < 6; i++) {
        if (mask[i] && !(vector1[i] <= vector2[i])) {
            return false;
        }
    }
    return true;
}

bool isGreater(const VectorXd& vector1, const VectorXd& vector2) {
    if (vector1.size() != vector2.size()) {
        throw std::invalid_argument("Function isBetween can only take arguments with the same "
            "number of elements.");
    }
    for (size_t i = 0; i < vector2.size(); i++) {
        if (!(vector1[i] >= vector2[i])) {
            return false;
        }
    }
    return true;
}

bool isGreater(const Vector6d& vector1, const Vector6d& vector2, const TaskSpace& mask) {
    for (size_t i = 0; i < 6; i++) {
        if (mask[i] && !(vector1[i] >= vector2[i])) {
            return false;
        }
    }
    return true;
}

}  // namespace crf::utility::types
