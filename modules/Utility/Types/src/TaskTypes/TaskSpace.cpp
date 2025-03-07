/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include "Types/TaskTypes/TaskSpace.hpp"

namespace crf::utility::types {

TaskSpace::TaskSpace() {
    for (size_t i = 0; i < 6; i++) {
        (*this)[i] = true;
    }
}

TaskSpace::TaskSpace(const std::map<TaskSpaceTangentDimension, bool>& taskSpace) :
    taskSpace_(taskSpace) {
}

TaskSpace::TaskSpace(const std::array<bool, 6>& taskSpace) {
    for (size_t i = 0; i < 6; i++) {
        (*this)[i] = taskSpace[i];
    }
}

TaskSpace::TaskSpace(const std::initializer_list<bool>& taskSpace) {
    if (taskSpace.size() != 6) {
        throw std::invalid_argument(
            "TaskSpace can only be constructed from initialiser list of 6 elements.");
    } else {
        std::vector<bool> stdVectorTaskSpace(taskSpace);
        for (int i = 0; i < 6; i++) {
            (*this)[i] = stdVectorTaskSpace[i];
        }
    }
}

TaskSpace& TaskSpace::operator=(const std::map<TaskSpaceTangentDimension, bool>& taskSpace) {
    taskSpace_ = taskSpace;
    return *this;
}

TaskSpace& TaskSpace::operator=(const std::array<bool, 6>& taskSpace) {
    for (int i = 0; i < 6; i++) {
        (*this)[i] = taskSpace[i];
    }
    return *this;
}

TaskSpace& TaskSpace::operator=(const std::initializer_list<bool>& taskSpace) {
    if (taskSpace.size() != 6) {
        throw std::invalid_argument(
            "TaskSpace can only be assign from initialiser list of 6 elements.");
    } else {
        std::vector<bool> stdVectorTaskSpace(taskSpace);
        for (int i = 0; i < 6; i++) {
            (*this)[i] = stdVectorTaskSpace[i];
        }
    }
    return *this;
}

size_t TaskSpace::dimension() const {
    size_t dimension = 0;
    for (size_t i = 0; i < 6; i++) {
        if ((*this)[i]) {
            dimension++;
        }
    }
    return dimension;
}

size_t TaskSpace::linearDimension() const {
    size_t dimension = 0;
    for (size_t i = 0; i < 3; i++) {
        if ((*this)[i]) {
            dimension++;
        }
    }
    return dimension;
}

size_t TaskSpace::angularDimension() const {
    size_t dimension = 0;
    for (size_t i = 3; i < 6; i++) {
        if ((*this)[i]) {
            dimension++;
        }
    }
    return dimension;
}

Eigen::Matrix<double, Eigen::Dynamic, 6> TaskSpace::getNoRowsMatrix() const {
    Eigen::Matrix<double, Eigen::Dynamic, 6> noRowsMatrix;
    size_t noOfRows = 0;
    for (size_t i = 0; i < 6; i++) {
        if ((*this)[i]) {
            noOfRows++;
        }
    }
    noRowsMatrix.resize(noOfRows, 6);
    noRowsMatrix = Eigen::Matrix<double, Eigen::Dynamic, 6>::Zero(noOfRows, 6);
    size_t rowCount = 0;
    for (size_t i = 0; i < 6; i++) {
        if ((*this)[i]) {
            noRowsMatrix(rowCount, i) = 1.0;
            rowCount++;
        }
    }
    return noRowsMatrix;
}

Eigen::Matrix<double, 6, 6> TaskSpace::getZeroRowsMatrix() const {
    Eigen::Matrix<double, 6, 6> zeroRowsMatrix(Eigen::Matrix<double, 6, 6>::Zero());
    for (size_t i = 0; i < 6; i++) {
        if ((*this)[i]) {
            zeroRowsMatrix(i, i) = 1.0;
        }
    }
    return zeroRowsMatrix;
}

Eigen::Matrix<double, 6, 6> TaskSpace::getNaNRowsMatrix() const {
    Eigen::Matrix<double, 6, 6> NaNRowsMatrix(Eigen::Matrix<double, 6, 6>::Zero());
    for (size_t i = 0; i < 6; i++) {
        if ((*this)[i]) {
            NaNRowsMatrix(i, i) = 1.0;
        } else {
            for (size_t j = 0; j < 6; j++) {
                NaNRowsMatrix(i, j) = std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    return NaNRowsMatrix;
}

Eigen::Vector<double, 6> TaskSpace::getVector6d() const {
    Eigen::Vector<double, 6> vector;
    for (size_t i = 0; i < 6; i++) {
        if ((*this)[i]) {
            vector(i) = 1.0;
        } else {
            vector(i) = 0.0;
        }
    }
    return vector;
}

std::size_t TaskSpace::size() const {
    return static_cast<std::size_t>(6);
}

bool& TaskSpace::operator[](const TaskSpaceTangentDimension& dimension) {
    return taskSpace_[dimension];
}

bool TaskSpace::operator[](const TaskSpaceTangentDimension& dimension) const {
    return taskSpace_.at(dimension);
}

bool& TaskSpace::operator[](const size_t& index) {
    switch (index) {
        case 0:
            return taskSpace_[TaskSpaceTangentDimension::Vx];
            break;
        case 1:
            return taskSpace_[TaskSpaceTangentDimension::Vy];
            break;
        case 2:
            return taskSpace_[TaskSpaceTangentDimension::Vz];
            break;
        case 3:
            return taskSpace_[TaskSpaceTangentDimension::Wx];
            break;
        case 4:
            return taskSpace_[TaskSpaceTangentDimension::Wy];
            break;
        case 5:
            return taskSpace_[TaskSpaceTangentDimension::Wz];
            break;
        default:
            throw std::out_of_range(
                "bool TaskSpace::operator[](size_t): Dimension can be only from 0 to 5.");
            break;
    }
    throw std::logic_error(
        "bool TaskSpace::operator[](size_t): Control reached an impossible point.");
}

bool TaskSpace::operator[](const size_t& index) const {
    switch (index) {
        case 0:
            return taskSpace_.at(TaskSpaceTangentDimension::Vx);
            break;
        case 1:
            return taskSpace_.at(TaskSpaceTangentDimension::Vy);
            break;
        case 2:
            return taskSpace_.at(TaskSpaceTangentDimension::Vz);
            break;
        case 3:
            return taskSpace_.at(TaskSpaceTangentDimension::Wx);
            break;
        case 4:
            return taskSpace_.at(TaskSpaceTangentDimension::Wy);
            break;
        case 5:
            return taskSpace_.at(TaskSpaceTangentDimension::Wz);
            break;
        default:
            throw std::out_of_range(
                "bool TaskSpace::operator[](size_t) const: Dimension can be only from 0 to 5.");
            break;
    }
    throw std::logic_error(
        "bool TaskSpace::operator[](size_t) const: Control reached an impossible point.");
}

std::ostream& operator<<(std::ostream& os, const TaskSpace& taskSpace) {
    std::array<std::string, 6> taskSpaceTangentDimension({"Vx", "Vy", "Vz", "Wx", "Wy", "Wz"});
    os << std::fixed << std::setprecision(15);
    for (std::size_t i = 0; i < taskSpace.size(); i++) {
        if (i > 0) {
            os << "\n";
        }
        os << taskSpaceTangentDimension[i] << ": ";
        if (taskSpace[i]) {
            os << "true";
        } else {
            os << "false";
        }
    }
    return os;
}

}  // namespace crf::utility::types
