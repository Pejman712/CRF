/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021
 *
 *  ================================================================================================================
*/

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include "ForwardKinematics/MathExprForwardKinematics/MathExprForwardKinematics.hpp"

using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::math::rotation::CardanXYZ;

namespace crf::control::forwardkinematics {

MathExprForwardKinematics::MathExprForwardKinematics(nlohmann::json jsonConfig,
    std::vector<double> lx,
    std::vector<double> ly,
    std::vector<double> lz):
    logger_("MathExprForwardKinematics"),
    jsonConfig_(jsonConfig),
    lx_(lx),
    ly_(ly),
    lz_(lz) {
    logger_->debug("CTor");
    if (jsonConfig_.size() == 0) {
        throw std::runtime_error("The json input file is empty");
    }
    numberOfJoints_ = lx_.size();
    if (numberOfJoints_ != ly_.size() || numberOfJoints_ != lz_.size()) {
        throw std::runtime_error("Input parameters not valid");
    }
    q_.resize(numberOfJoints_);
    qd_.resize(numberOfJoints_);
    qdd_.resize(numberOfJoints_);
    for (unsigned int i = 0; i < numberOfJoints_; i++) {
        q_[i] = 0.0;
        qd_[i] = 0.0;
        qdd_[i] = 0.0;
        symbolTable_.add_variable("q" + std::to_string(i + 1), q_[i]);
        symbolTable_.add_variable("qd" + std::to_string(i + 1), qd_[i]);
        symbolTable_.add_variable("qdd" + std::to_string(i + 1), qdd_[i]);
        symbolTable_.add_variable("l" + std::to_string(i + 1) + "x", lx_[i]);
        symbolTable_.add_variable("l" + std::to_string(i + 1) + "y", ly_[i]);
        symbolTable_.add_variable("l" + std::to_string(i + 1) + "z", lz_[i]);
    }
    poseExpression_.register_symbol_table(symbolTable_);
    velocityExpression_.register_symbol_table(symbolTable_);
    accelerationExpression_.register_symbol_table(symbolTable_);
    sizeJSONFile_ = 0;
    isTherePosition_ = false;
    isThereVelocity_ = false;
    isThereAcceleration_ = false;
    std::optional<exprtk::expression<double>> result;
    result = parse(poseExpression_,
    "IrIE", "RIE");
    if (result) {
        isTherePosition_ = true;
        poseExpression_ = result.value();
    } else {
        result = parse(poseExpression_,
         "IrIE", "Quaternion");
        if (result) {
            isTherePosition_ = true;
            poseExpression_ = result.value();
        } else {
            result = parse(poseExpression_, "IrIE", "CardanXYZ");
            if (result) {
                isTherePosition_ = true;
                poseExpression_ = result.value();
            }
        }
    }
    result = parse(velocityExpression_,
     "IvIE", "IwIE");
    if (result) {
        isThereVelocity_ = true;
        velocityExpression_ = result.value();
    }
    result = parse(accelerationExpression_,
     "IaIE", "IalphaIE");
    if (result) {
        isThereAcceleration_ = true;
        accelerationExpression_ = result.value();
    }
    if (jsonConfig_.size() != sizeJSONFile_) {
        throw std::runtime_error("One or more matrix/vectors are empty or have a wrong name");
    }
}

MathExprForwardKinematics::~MathExprForwardKinematics() {
    logger_->debug("DTor");
}

std::optional<TaskPose> MathExprForwardKinematics::getPose(
    const JointPositions& jointPositions) {
    logger_->debug("getPose");
    if (!isTherePosition_) {
        logger_->error("RIE/quaternion, IrIE or CustomPoseIE not found in the configuration file");
        return std::nullopt;
    }
    if (jointPositions.size() != numberOfJoints_) {
        logger_->error("The dimension of the joints position ({}) does not match ({}).",
            jointPositions.size(), numberOfJoints_);
        return std::nullopt;
    }
    for (unsigned int i = 0; i < numberOfJoints_; i++) {
        q_[i] = jointPositions[i];
    }
    poseExpression_.value();
    Eigen::Vector3d IrIE({IrIE_[0], IrIE_[1], IrIE_[2]});
    Eigen::Matrix3d RIE;
    Eigen::Quaternion<double> quaternion;
    CardanXYZ cardanXYZ;
    switch (orientationRepresentation_) {
        case OrientationRepresentation::Matrix:
            RIE << RIE_[0], RIE_[1], RIE_[2], RIE_[3], RIE_[4], RIE_[5], RIE_[6], RIE_[7], RIE_[8];
            return TaskPose(IrIE, RIE);
            break;
        case OrientationRepresentation::Quaternion:
            quaternion = {quaternion_[0], quaternion_[1],
            quaternion_[2], quaternion_[3]};
            return TaskPose(IrIE, quaternion);
            break;
        case OrientationRepresentation::CardanXYZ:
            cardanXYZ = CardanXYZ({cardanXYZ_[0], cardanXYZ_[1], cardanXYZ_[2]});
            return TaskPose(IrIE, cardanXYZ);
            break;
        default:
            throw(
                std::logic_error(
                    "MathExprForwardKinematics::getPose: orientationRepresentation_ is invalid;"));
            break;
    }
}

std::optional<TaskVelocity> MathExprForwardKinematics::getVelocity(
    const JointPositions& jointPositions,
    const JointVelocities& jointVelocities) {
    logger_->debug("getVelocity");
    if (!isThereVelocity_) {
        logger_->error("IwIE, IvIE or CustomVelIE not found in the configuration file");
        return std::nullopt;
    }
    if (jointPositions.size() != numberOfJoints_) {
        logger_->error("The dimension of the joints position ({}) does not match ({}).",
            jointPositions.size(), numberOfJoints_);
        return std::nullopt;
    }
    if (jointVelocities.size() != numberOfJoints_) {
        logger_->error("The dimension of the joints velocity ({}) does not match ({}).",
            jointVelocities.size(), numberOfJoints_);
        return std::nullopt;
    }
    for (unsigned int i = 0; i < numberOfJoints_; i++) {
        q_[i] = jointPositions[i];
        qd_[i] = jointVelocities[i];
    }
    velocityExpression_.value();
    return TaskVelocity({IvIE_[0], IvIE_[1], IvIE_[2], IwIE_[0], IwIE_[1], IwIE_[2]});
}

std::optional<TaskAcceleration> MathExprForwardKinematics::getAcceleration(
    const JointPositions& jointPositions,
    const JointVelocities& jointVelocities,
    const JointAccelerations& jointAccelerations) {
    logger_->debug("getAcceleration");
    if (!isThereAcceleration_) {
        logger_->error("IalphaIE, IaIE or CustomAccIE not found in the configuration file");
        return std::nullopt;
    }
    if (jointPositions.size() != numberOfJoints_) {
        logger_->error("The dimension of the joints position ({}) does not match ({}).",
            jointPositions.size(), numberOfJoints_);
        return std::nullopt;
    }
    if (jointVelocities.size() != numberOfJoints_) {
        logger_->error("The dimension of the joints velocity ({}) does not match ({}).",
            jointVelocities.size(), numberOfJoints_);
        return std::nullopt;
    }
    if (jointAccelerations.size() != numberOfJoints_) {
        logger_->error("The dimension of the joints acceleration ({}) does not match ({}).",
            jointAccelerations.size(), numberOfJoints_);
        return std::nullopt;
    }
    for (unsigned int i = 0; i < numberOfJoints_; i++) {
        q_[i] = jointPositions[i];
        qd_[i] = jointVelocities[i];
        qdd_[i] = jointAccelerations[i];
    }
    accelerationExpression_.value();
    return TaskAcceleration({IaIE_[0], IaIE_[1], IaIE_[2],
        IalphaIE_[0], IalphaIE_[1], IalphaIE_[2]});
}

std::optional<exprtk::expression<double>> MathExprForwardKinematics::parse(
    exprtk::expression<double> expression,
    const std::string linear,
    const std::string angular) {
    if (!jsonConfig_.contains(angular) && !jsonConfig_.contains(linear)) {
        return std::nullopt;
    } else if (!jsonConfig_.contains(angular) || !jsonConfig_.contains(linear)) {
        sizeJSONFile_++;
        return std::nullopt;
    }
    sizeJSONFile_+=2;
    if (jsonConfig_[angular].size() == 0 || jsonConfig_[linear].size() == 0) {
        return std::nullopt;
    }
    if (angular == "RIE") {
        orientationRepresentation_ = OrientationRepresentation::Matrix;
        RIE_.resize(9);
        symbolTable_.add_vector("RIE", RIE_);
        IrIE_.resize(3);
        symbolTable_.add_vector("IrIE", IrIE_);
        poseExpression_.register_symbol_table(symbolTable_);
    } else if (angular == "Quaternion") {
        orientationRepresentation_ = OrientationRepresentation::Quaternion;
        quaternion_.resize(4);
        symbolTable_.add_vector("Quaternion", quaternion_);
        IrIE_.resize(3);
        symbolTable_.add_vector("IrIE", IrIE_);
        poseExpression_.register_symbol_table(symbolTable_);
    } else if (angular == "CardanXYZ") {
        orientationRepresentation_ = OrientationRepresentation::CardanXYZ;
        cardanXYZ_.resize(3);
        symbolTable_.add_vector("CardanXYZ", cardanXYZ_);
        IrIE_.resize(3);
        symbolTable_.add_vector("IrIE", IrIE_);
        poseExpression_.register_symbol_table(symbolTable_);
    } else if (angular == "IwIE") {
        IwIE_.resize(3);
        symbolTable_.add_vector("IwIE", IwIE_);
        IvIE_.resize(3);
        symbolTable_.add_vector("IvIE", IvIE_);
        velocityExpression_.register_symbol_table(symbolTable_);
    } else if (angular == "IalphaIE") {
        IalphaIE_.resize(3);
        symbolTable_.add_vector("IalphaIE", IalphaIE_);
        IaIE_.resize(3);
        symbolTable_.add_vector("IaIE", IaIE_);
        accelerationExpression_.register_symbol_table(symbolTable_);
    } else {
        throw std::invalid_argument(
            "crf::control::forwardkinematics::MathExprForwardKinematics::parse"
            "angular has not a proper value.");
    }
    int maxNumber = 0;
    int sizeAngular = 0;
    if (angular == "Quaternion") {
        // It already sum in sizeJSONFile the linear when this function passed before as "RIE",
        // so we substract 1
        sizeJSONFile_--;
        if (jsonConfig_[angular].size() != 4) {
            throw std::runtime_error("The size of " + angular + " should be 4");
        }
        sizeAngular = 4;
    } else if (angular == "CardanXYZ") {
        // It already sum in sizeJSONFile the linear twice when this function
        // passed before as "RIE" and as a quaternion, so we substract 2
        sizeJSONFile_-= 2;
        if (jsonConfig_[angular].size() != 3) {
            throw std::runtime_error("The size of " + angular + " should be 3 "
                "(3 cardan angles)");
        }
        sizeAngular = 3;
    } else {
        if (jsonConfig_[angular].size() != 3) {
            throw std::runtime_error("The size of " + angular + " should be 3 "
                "(3 rows of the rotation matrix)");
        }
        sizeAngular = 3;
    }
    for (int row = 0; row < sizeAngular; row++) {
        if (angular == "RIE") {
            if (!jsonConfig_[angular].contains("Row" + std::to_string(row))) {
                throw std::runtime_error("The name of one or more rows in " + angular +
                    " is wrong");
            }
            if (jsonConfig_[angular]["Row" + std::to_string(row)].size() != 3) {
                throw std::runtime_error("The size of " + angular + "[" + std::to_string(row) +
                    "] should be 3 (3 columns of the rotation matrix)");
            }
        } else if (angular == "CardanXYZ") {
            std::array<std::string, 3> axes({"X", "Y", "Z"});
            if (!jsonConfig_[angular].contains(axes[row])) {
                throw std::runtime_error("The name of one or more components in " + angular +
                    " is wrong");
            }
            if (jsonConfig_[angular][axes[row]].size() != 1) {
                throw std::runtime_error(angular + "[" + axes[row] +
                    "] should be only 1 expression.");
            }
        } else {
            if (!jsonConfig_[angular].contains(std::to_string(row))) {
                throw std::runtime_error("The name of one or more components in " + angular +
                    " is wrong");
            }
            if (jsonConfig_[angular][std::to_string(row)].size() != 1) {
                throw std::runtime_error(angular + "[" + std::to_string(row) +
                    "] should be only 1 expression.");
            }
        }
    }
    if (jsonConfig_[linear].size() != 3) {
        throw std::runtime_error("The size of " + linear + " should be 3 "
            "(3 components of the traslation vector)");
    }
    maxNumber = sizeAngular;

    std::string mathExpression;
    int contPos = 0;
    for (int row = 0; row < maxNumber; row++) {
        if (angular == "RIE") {
            for (int column = 0; column < 3; column++) {
                if (!jsonConfig_[angular]["Row" + std::to_string(row)].contains("Col" + std::to_string(column))) {  // NOLINT
                    throw std::runtime_error("The name of one or more columns in " + angular +
                        "[" + std::to_string(row) + "] is wrong");
                }
                mathExpression += angular;
                mathExpression += "[";
                mathExpression += std::to_string(contPos);
                mathExpression += "] := ";
                mathExpression += ifFromNaN(jsonConfig_[angular]["Row" + std::to_string(row)]["Col" + std::to_string(column)]);  // NOLINT
                mathExpression += ";\n";
                contPos++;
            }
        } else if (angular == "CardanXYZ") {
            std::array<std::string, 3> axes({"X", "Y", "Z"});
            mathExpression += angular;
            mathExpression += "[";
            mathExpression += std::to_string(row);
            mathExpression += "] := ";
            mathExpression += ifFromNaN(jsonConfig_[angular][axes[row]]);
            mathExpression += ";\n";
            contPos++;
        } else {
            mathExpression += angular;
            mathExpression += "[";
            mathExpression += std::to_string(row);
            mathExpression += "] := ";
            mathExpression += ifFromNaN(jsonConfig_[angular][std::to_string(row)]);
            mathExpression += ";\n";
            contPos++;
        }
        if (row == 3) {
            break;
        }
        if (!jsonConfig_[linear].contains(std::to_string(row))) {
            throw std::runtime_error("The name of one or more components in " + linear +
                " is wrong");
        }
        if (jsonConfig_[linear][std::to_string(row)].size() != 1) {
            throw std::runtime_error(linear + "[" + std::to_string(row) +
                "] should be only 1 expression.");
        }
        mathExpression += linear;
        mathExpression += "[";
        mathExpression += std::to_string(row);
        mathExpression += "] := ";
        mathExpression += ifFromNaN(jsonConfig_[linear][std::to_string(row)]);
        mathExpression += ";\n";
    }
    exprtk::parser<double> parser;
    if (!parser.compile(mathExpression, expression)) {
        for (std::size_t i = 0; i < parser.error_count(); ++i) {
            exprtk::parser_error::type error = parser.get_error(i);
            throw std::runtime_error("Error[" + std::to_string(i) + "] Position: " +
                std::to_string(error.token.position) + " Type: [" +
                exprtk::parser_error::to_str(error.mode).c_str() + "] Msg: " +
                error.diagnostic.c_str() + "\nError parsing" +
                linear + " and " + angular);
        }
    }
    return expression;
}

std::string ifFromNaN(const std::string& input) {
    if (input == "NaN") {
        return "if(false){}";
    } else {
        return input;
    }
}

}  // namespace crf::control::forwardkinematics
