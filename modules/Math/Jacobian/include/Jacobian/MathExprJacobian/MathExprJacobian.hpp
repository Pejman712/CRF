/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
 */

#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>
#include <memory>

#include <nlohmann/json.hpp>

#include "Jacobian/IJacobian.hpp"
#include "MathExprTk/exprtk.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf::math::jacobian {

/**
 * @ingroup group_math_expressions_jacobian
 * @brief Implementation of IJacobian. This implementation constructs a Jacobian of
 * a robot with a set of Mathematical expressions
 *
 */
class MathExprJacobian : public IJacobian {
 public:
    /**
     * @brief Construct a new Jacobian object reading the Jacobian matrix in string format
     *
     * @param mathExpression is a string variable that contains the mathematical Jacobian matrix
     *        expressions
     * @param lxValues is the vector of lengths between the robot joints in the X axis
     * @param lyValues is the vector of lengths between the robot joints in the Y axis
     * @param lzValues is the vector of lengths between the robot joints in the Z axis
     */
    explicit MathExprJacobian(const nlohmann::json &jsonConfig, const std::vector<double>& lxValues,
      const std::vector<double>& lyValues, const std::vector<double>& lzValues);

    /**
     * @brief Destroy the Jacobian object
     */
    ~MathExprJacobian() override;

    /**
     * @brief Calculates the numerical values of the Jacobian Matrix for specified joint angles
     *
     * @param qValues contains the joints position values
     * @return Numeric jacobian matrix
     */
    Eigen::MatrixXd evaluate(const JointPositions& qValues) override;
    /**
     * @brief It computes the kinematic manipulability parameter for an especific joints position
     *        values
     *
     * @param qValues contains the joints position values
     * @return Kinematic manipulability parameter
     * @return std::nullopt on failure
     */
    double getKinematicManipulability(const JointPositions& qValues) override;
    /**
     * @brief It gives the number of rows of the Jacobian matrix
     *
     * @return number of rows
     */
    unsigned int rows() const override;
    /**
     * @brief It gives the number of columns of the Jacobian matrix
     *
     * @return number of columns
     */
    unsigned int cols() const override;

    /**
     * @brief Printable format of Jacobian
     */
    friend std::ostream& operator<<(std::ostream& os, const MathExprJacobian& j);


 private:
    utility::logger::EventLogger logger_;

    nlohmann::json jsonConfig_;
    std::string mathExpression_;
    std::vector<double> lx_;
    std::vector<double> ly_;
    std::vector<double> lz_;
    unsigned int columnsJMatrix_;
    unsigned int rowsJMatrix_;
    Eigen::MatrixXd JMatrix_;
    std::vector<double> q_;
    std::vector<double> J_;
    exprtk::symbol_table<double> symbolTable_;
    exprtk::expression<double> expression_;

    /**
     * @brief Do de conversion of the jacobian mathematical expressions form json to string
     *        with the necessary modifications for the parser can understand it.
     *        This function is used in the constructor that reads the Jacobian Matrix
     *
     * @return String with the jacobian mathematical expressions for parser understanding
     */
    std::string transformJson2String();
    /**
     * @brief
     */
    void createMathExpression();
};

}  // namespace crf::math::jacobian
