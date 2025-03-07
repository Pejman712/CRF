/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include "Jacobian/MathExprJacobian/MathExprJacobian.hpp"

namespace crf::math::jacobian {

MathExprJacobian::MathExprJacobian(const nlohmann::json &jsonConfig,
    const std::vector<double>& lxValues,
    const std::vector<double>& lyValues,
    const std::vector<double>& lzValues):
    logger_("MathExprJacobian"),
    jsonConfig_(jsonConfig),
    lx_(lxValues),
    ly_(lyValues),
    lz_(lzValues) {
    logger_->debug("CTor");
    // There is no way to check if Jacobian expression has the correct number of rows.
    rowsJMatrix_ = jsonConfig_.size();
    if (jsonConfig_.size() == 0) {
        throw std::invalid_argument("The json input file is empty");
    }
    if (lx_.size() != ly_.size() || lx_.size() != lz_.size()) {
        throw std::invalid_argument("Input parameters not valid");
    }
    columnsJMatrix_ = lx_.size();
    mathExpression_ = transformJson2String();
    if (rowsJMatrix_ > columnsJMatrix_) {
        throw std::invalid_argument("Jacobian can't contain more rows than columns "
            "(more space dimensions than robot DOF)");
    }
    JMatrix_ = Eigen::MatrixXd(rowsJMatrix_, columnsJMatrix_);
    q_.resize(columnsJMatrix_);
    J_.resize(rowsJMatrix_*columnsJMatrix_);
    createMathExpression();
}

MathExprJacobian::~MathExprJacobian() {
    logger_->debug("DTor");
}

Eigen::MatrixXd MathExprJacobian::evaluate(const JointPositions& qValues) {
    logger_->debug("evaluate {}", qValues);
    if (qValues.size() != columnsJMatrix_) {
        logger_->error("The dimension of q vector ({}) is different than the length vectors "\
            "defined in the constructor ({}).", qValues.size(), columnsJMatrix_);
        throw std::invalid_argument(
            "MathExprJacobian - evaluate - The dimensions of the input joint position "
            "does not match the jacobian dimensions");
    }
    for (unsigned int i = 0; i < qValues.size(); i++) {
        q_[i] = qValues[i];
    }
    expression_.value();
    unsigned int cont = 0;
    for (unsigned int i = 0; i < rowsJMatrix_; i++) {
        for (unsigned int j = 0; j < columnsJMatrix_; j++) {
            JMatrix_(i, j) = J_[cont];
            cont++;
        }
    }
    return JMatrix_;
}

double MathExprJacobian::getKinematicManipulability(const JointPositions& qValues) {
    logger_->debug("getKinematicManipulability");
    if (qValues.size() != columnsJMatrix_) {
        logger_->error("The dimension of q vector ({}) is different than the length vectors "\
            "defined in the constructor ({}).", qValues.size(), columnsJMatrix_);
        throw std::invalid_argument(
            "MathExprJacobian - getKinematicManipulability - The dimensions of the input "
            "joint position does not match the jacobian dimensions");
    }
    Eigen::MatrixXd j = evaluate(qValues);
    return std::sqrt((j*j.transpose()).determinant());
}

unsigned int MathExprJacobian::rows() const {
    return rowsJMatrix_;
}

unsigned int MathExprJacobian::cols() const {
    return columnsJMatrix_;
}

std::ostream& operator<<(std::ostream& os, const MathExprJacobian& J) {
    std::string stringJ;
    for (unsigned int row = 0; row < J.rowsJMatrix_; row++) {
        for (unsigned int column = 0; column < J.columnsJMatrix_; column++) {
            stringJ += "J[";
            stringJ += std::to_string(row);
            stringJ += "][";
            stringJ += std::to_string(column);
            stringJ += "] = ";
            stringJ += J.jsonConfig_["Row" + std::to_string(row)]["Col" + std::to_string(column)];
            stringJ += ";\n";
        }
    }

    int i = 0;
    for (unsigned int j = 0; j < (J.rowsJMatrix_*J.columnsJMatrix_); j++) {
        while (stringJ[i] != ';') {
            os << stringJ[i];
            i++;
        }
        os << '\n';
        i++;
    }
    return os;
}

// Private

std::string MathExprJacobian::transformJson2String() {
    // Creation of the string Jacobian expression
    std::string mathExpression;
    int contPos = 0;
    for (unsigned int row = 0; row < rowsJMatrix_; row++) {
        // Check the row number
        if (!jsonConfig_.contains("Row" + std::to_string(row))) {
            throw std::invalid_argument("The name of one or more rows in Jacobian is wrong");
        }
        // Check if Jacobian has the correct number of columns in each row.
        if (jsonConfig_["Row" + std::to_string(row)].size() != columnsJMatrix_) {
            throw std::invalid_argument("Row: " + std::to_string(row) + " -> The number of colums "
                "of Jacobian matrix should be " + std::to_string(columnsJMatrix_));
        }
        for (unsigned int column = 0; column < columnsJMatrix_; column++) {
            // Check the column number
            if (!jsonConfig_["Row" + std::to_string(row)].contains("Col" + std::to_string(column))) {  // NOLINT
                throw std::invalid_argument("The name of one or more columns in Jacobian[" +
                    std::to_string(row) + "] is wrong");
            }
            // Check that there is only one expression in each element of the jacobian matrix
            if (jsonConfig_["Row" + std::to_string(row)]["Col" + std::to_string(column)].size() != 1) {  // NOLINT
                throw std::invalid_argument("There should be only one expression in each element "
                    "of the jacobian matrix. "
                    "Check Jacobian[" + std::to_string(row) + "][" + std::to_string(column) + "]");
            }
            // Creation of the string Jacobian expression
            mathExpression += "J[";
            mathExpression += std::to_string(contPos);
            mathExpression += "] := ";
            mathExpression +=
                jsonConfig_["Row" + std::to_string(row)]["Col" + std::to_string(column)];
            mathExpression += ";\n";
            contPos++;
        }
    }
    return mathExpression;
}

void MathExprJacobian::createMathExpression() {
    for (unsigned int i = 0; i < columnsJMatrix_; i++) {
        symbolTable_.add_variable("q" + std::to_string(i+1), q_[i]);
        symbolTable_.add_variable("l" + std::to_string(i+1) + "x", lx_[i]);
        symbolTable_.add_variable("l" + std::to_string(i+1) + "y", ly_[i]);
        symbolTable_.add_variable("l" + std::to_string(i+1) + "z", lz_[i]);
    }
    symbolTable_.add_vector("J", J_);
    expression_.register_symbol_table(symbolTable_);
    exprtk::parser<double> parser;
    if (!parser.compile(mathExpression_, expression_)) {
        for (std::size_t i = 0; i < parser.error_count(); ++i) {
            exprtk::parser_error::type error = parser.get_error(i);
            throw std::invalid_argument("Error[" + std::to_string(i) + "] Position: " +
                std::to_string(error.token.position) + " Type: [" +
                exprtk::parser_error::to_str(error.mode).c_str() + "] Msg: " +
                error.diagnostic.c_str());
        }
    }
}

}  // namespace crf::math::jacobian
