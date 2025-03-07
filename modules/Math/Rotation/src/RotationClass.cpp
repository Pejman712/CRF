/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#include <Eigen/Dense>

#include <iostream>
#include <sstream>

#include "Rotation/RotationClass.hpp"

namespace crf::math::rotation {

Rotation::Rotation() :
    quaternion_(Eigen::Quaterniond({1.0, 0.0, 0.0, 0.0})),
    representation_(RotationRepresentation::Quaternion) {
}

Rotation::Rotation(const Eigen::Quaterniond& quaternion, const double& accuracy) :
    quaternion_(quaternion),
    representation_(RotationRepresentation::Quaternion) {
    if (!isUnitaryQuaternion(quaternion_, accuracy)) {
        double norm = quaternion.norm();
        std::stringstream throwStringStream;
        throwStringStream << std::setprecision(16);
        throwStringStream
            << "Rotation(Eigen::Quaterniond): quaternion provided in the constructor is not "
               "up to accuracy a unit quaternion. It has norm equal to "
            << norm << " which differs from " << 1.0 << " by " << abs(1 - norm)
            << " but set accuracy is " << accuracy << ".";
        throw std::invalid_argument(throwStringStream.str());
    }
}

Rotation::Rotation(const Eigen::Matrix3d& matrix, const double& accuracy) :
    matrix_(matrix),
    representation_(RotationRepresentation::Matrix) {
    if (!isRotationMatrix(matrix_, accuracy)) {
        double determinant = matrix.determinant();
        Eigen::Matrix3d symmetrisation = matrix * matrix.transpose();
        double normOfDifference =
            (matrix * matrix.transpose() - Eigen::Matrix3d::Identity()).lpNorm<Eigen::Infinity>();
        std::stringstream throwStringStream;
        throwStringStream << std::setprecision(16);
        throwStringStream
            << "Rotation(Eigen::Matrix3d): Matrix provided in the constructor is not close "
               "up to accuracy a rotation matrix. It's determinant is equal to "
            << determinant << " which differs from 1 by " << abs(1 - determinant)
            << ".\nmatrix*matrix.transpose() is equal to\n"
            << symmetrisation
            << ",\n which difference from identity matrix has linfty norm equal to "
            << normOfDifference << ", but set accuracy is " << accuracy << ".";
        throw std::invalid_argument(throwStringStream.str());
    }
}

Rotation::Rotation(const Eigen::AngleAxisd& angleAxis) :
    angleAxis_(angleAxis),
    representation_(RotationRepresentation::AngleAxis) {
}

Rotation::Rotation(const CardanXYZ& cardanXYZ) :
    cardanXYZ_(cardanXYZ),
    representation_(RotationRepresentation::CardanXYZ) {
}

Rotation::Rotation(const EulerZXZ& eulerZXZ) :
    eulerZXZ_(eulerZXZ),
    representation_(RotationRepresentation::EulerZXZ) {
}

void Rotation::setQuaternion(const Eigen::Quaterniond& quaternion, const double& accuracy) {
    quaternion_ = quaternion;
    representation_ = RotationRepresentation::Quaternion;
    if (!isUnitaryQuaternion(quaternion_, accuracy)) {
        double norm = quaternion_.norm();
        std::stringstream throwStringStream;
        throwStringStream << std::setprecision(16);
        throwStringStream << "Rotation::setQuaternion: quaternion provided in the argument is not "
                             "up to accuracy a unit quaternion. It has norm equal to "
                          << norm << " which differs from 1 by " << abs(1 - norm)
                          << " but set accuracy is " << accuracy << ".";
        throw std::invalid_argument(throwStringStream.str());
    }
}

void Rotation::setMatrix(const Eigen::Matrix3d& matrix, const double& accuracy) {
    matrix_ = matrix;
    representation_ = RotationRepresentation::Matrix;
    if (!isRotationMatrix(matrix_, accuracy)) {
        double determinant = matrix.determinant();
        Eigen::Matrix3d symmetrisation = matrix * matrix.transpose();
        double normOfDifference =
            (matrix * matrix.transpose() - Eigen::Matrix3d::Identity()).lpNorm<Eigen::Infinity>();
        std::stringstream throwStringStream;
        throwStringStream << std::setprecision(16);
        throwStringStream << "Rotation::setMatrix: Matrix provided in the argument is not "
                             "up to accuracy a rotation matrix. It's determinant is equal to "
                          << determinant << " which differs from 1 by " << abs(1 - determinant)
                          << ".\nmatrix*matrix.transpose() is equal to\n"
                          << symmetrisation
                          << ",\n which difference from identity matrix has linfty norm equal to "
                          << normOfDifference << ", but set accuracy is " << accuracy << ".";
        throw std::invalid_argument(throwStringStream.str());
    }
}

void Rotation::setAngleAxis(const Eigen::AngleAxisd& angleAxis) {
    angleAxis_ = angleAxis;
    representation_ = RotationRepresentation::AngleAxis;
}

void Rotation::setCardanXYZ(const CardanXYZ& cardanXYZ) {
    cardanXYZ_ = cardanXYZ;
    representation_ = RotationRepresentation::CardanXYZ;
}

void Rotation::setEulerZXZ(const EulerZXZ& eulerZXZ) {
    eulerZXZ_ = eulerZXZ;
    representation_ = RotationRepresentation::EulerZXZ;
}

Eigen::Quaterniond Rotation::getQuaternion() const {
    switch (representation_) {
        case RotationRepresentation::Quaternion:
            return quaternion_;

        case RotationRepresentation::Matrix:
            return quaternionFromMatrix(matrix_);

        case RotationRepresentation::AngleAxis:
            return quaternionFromAngleAxis(angleAxis_);

        case RotationRepresentation::CardanXYZ:
            return quaternionFromCardanXYZ(cardanXYZ_);

        case RotationRepresentation::EulerZXZ:
            return quaternionFromEulerZXZ(eulerZXZ_);

        default:
            throw std::logic_error("Rotation::getQuaternion: representation_ invalid");
    }
    throw std::logic_error("Rotation::getQuaternion: Control reached an impossible point.");
}

Eigen::Matrix3d Rotation::getMatrix() const {
    switch (representation_) {
        case RotationRepresentation::Quaternion:
            return matrixFromQuaternion(quaternion_);

        case RotationRepresentation::Matrix:
            return matrix_;

        case RotationRepresentation::AngleAxis:
            return matrixFromAngleAxis(angleAxis_);

        case RotationRepresentation::CardanXYZ:
            return matrixFromCardanXYZ(cardanXYZ_);

        case RotationRepresentation::EulerZXZ:
            return matrixFromEulerZXZ(eulerZXZ_);

        default:
            throw std::logic_error("Rotation::getMatrix: representation_ invalid");
    }
    throw std::logic_error("Rotation::getMatrix: Control reached an impossible point.");
}

Eigen::AngleAxisd Rotation::getAngleAxis() const {
    switch (representation_) {
        case RotationRepresentation::Quaternion:
            return angleAxisFromQuaternion(quaternion_);

        case RotationRepresentation::Matrix:
            return angleAxisFromMatrix(matrix_);

        case RotationRepresentation::AngleAxis:
            return angleAxis_;

        case RotationRepresentation::CardanXYZ:
            return angleAxisFromCardanXYZ(cardanXYZ_);

        case RotationRepresentation::EulerZXZ:
            return angleAxisFromEulerZXZ(eulerZXZ_);

        default:
            throw std::logic_error("Rotation::getAngleAxis: representation_ invalid");
    }
    throw std::logic_error("Rotation::getAngleAxis: Control reached an impossible point.");
}

CardanXYZ Rotation::getCardanXYZ() const {
    switch (representation_) {
        case RotationRepresentation::Quaternion:
            return cardanXYZFromQuaternion(quaternion_);

        case RotationRepresentation::Matrix:
            return cardanXYZFromMatrix(matrix_);

        case RotationRepresentation::AngleAxis:
            return cardanXYZFromAngleAxis(angleAxis_);

        case RotationRepresentation::CardanXYZ:
            return cardanXYZ_;

        case RotationRepresentation::EulerZXZ:
            return cardanXYZFromEulerZXZ(eulerZXZ_);

        default:
            throw std::logic_error("Rotation::getCardanXYZ: representation_ invalid");
    }
    throw std::logic_error("Rotation::getCardanXYZ: Control reached an impossible point.");
}

EulerZXZ Rotation::getEulerZXZ() const {
    switch (representation_) {
        case RotationRepresentation::Quaternion:
            return eulerZXZFromQuaternion(quaternion_);

        case RotationRepresentation::Matrix:
            return eulerZXZFromMatrix(matrix_);

        case RotationRepresentation::AngleAxis:
            return eulerZXZFromAngleAxis(angleAxis_);

        case RotationRepresentation::CardanXYZ:
            return eulerZXZFromCardanXYZ(cardanXYZ_);

        case RotationRepresentation::EulerZXZ:
            return eulerZXZ_;

        default:
            throw std::logic_error("Rotation::getEulerZXZ: representation_ invalid");
    }
    throw std::logic_error("Rotation::getEulerZXZ: Control reached an an impossible point.");
}

RotationRepresentation Rotation::getRepresentation() const {
    return representation_;
}

std::ostream& operator<<(std::ostream& os, const Rotation& rotation) {
    os << std::fixed << std::setprecision(15);
    const RotationRepresentation representation(rotation.getRepresentation());
    if (representation == RotationRepresentation::Quaternion) {
        Eigen::Quaterniond quaternion;
        bool someCoordinateIsNegative = false;
        quaternion = rotation.getQuaternion();
        if (quaternion.w() < 0) {
            someCoordinateIsNegative = true;
        }
        if (quaternion.x() < 0) {
            someCoordinateIsNegative = true;
        }
        if (quaternion.y() < 0) {
            someCoordinateIsNegative = true;
        }
        if (quaternion.z() < 0) {
            someCoordinateIsNegative = true;
        }
        os << "Quaternion:\n";
        os << "w: ";
        if (quaternion.w() >= 0 && someCoordinateIsNegative) {
            os << " ";
        }
        os << quaternion.w() << "\n";
        os << "x: ";
        if (quaternion.x() >= 0 && someCoordinateIsNegative) {
            os << " ";
        }
        os << quaternion.x() << "\n";
        os << "y: ";
        if (quaternion.y() >= 0 && someCoordinateIsNegative) {
            os << " ";
        }
        os << quaternion.y() << "\n";
        os << "z: ";
        if (quaternion.z() >= 0 && someCoordinateIsNegative) {
            os << " ";
        }
        os << quaternion.z();
    } else if (representation == RotationRepresentation::Matrix) {
        os << "Matrix:\n";
        os << rotation.getMatrix();
    } else if (representation == RotationRepresentation::AngleAxis) {
        Eigen::AngleAxisd angleAxis;
        angleAxis = rotation.getAngleAxis();
        os << "Angle:\n";
        os << angleAxis.angle() << "\n";
        os << "Axis:\n";
        os << angleAxis.axis();
    } else if (representation == RotationRepresentation::CardanXYZ) {
        os << "CardanXYZ:\n";
        os << rotation.getCardanXYZ();
    } else if (representation == RotationRepresentation::EulerZXZ) {
        os << "EulerZXZ:\n";
        os << rotation.getEulerZXZ();
    } else {
        throw std::logic_error("operator<<(std::ostream&, Rotation): representation_ invalid");
    }
    return os;
}

}  // namespace crf::math::rotation
