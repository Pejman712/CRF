/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>

#include <Eigen/Dense>

namespace crf::math::kalmanfilter {

/**
 * @ingroup group_kalman_filter
 * @brief Class to implement a linearized system model for the Extended Kalman Filter (EKF). In
 * this class we implement a default system model which corresponds to the g function returning the
 * previous mean and the jacobian G being the identity. In this implementation the EKF and the
 * standard kalamn filter are equivalent.
 *
 */
class SystemModelLinearized {
 public:
    explicit SystemModelLinearized(const Eigen::MatrixXd& noise);
    virtual ~SystemModelLinearized() = default;

    /**
     * @brief Method to evaluate the function g and the jacobian G
     *
     * @param mean Mean of the state space
     * @param control Control input
     */
    virtual void evaluate(const Eigen::VectorXd& mean, const Eigen::VectorXd& control);

    /**
     * @brief Getter for the function g evaluated
     *
     * @return Eigen::VectorXd Value of the function g
     */
    virtual Eigen::VectorXd g() const;

    /**
     * @brief Getter for the jacobian G evaluated
     *
     * @return Eigen::MatrixXd Values of the jacobian G
     */
    virtual Eigen::MatrixXd G() const;

    /**
     * @brief Getter for the noise of the system
     *
     * @return Eigen::MatrixXd Noise matrix
     */
    virtual Eigen::MatrixXd noise() const;

 protected:
    Eigen::VectorXd g_;
    Eigen::MatrixXd G_;
    Eigen::MatrixXd noise_;
    uint32_t dimensions_;
};

}  // namespace crf::math::kalmanfilter
