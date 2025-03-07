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
 * @brief Class to implement the system model for the Kalman Filter. In this class we implement a
 * default system model which corresponds to the identity matrix in A and B, and a control input of
 * 0. This basically takes the previous mean as the predicted mean of the Kalman Filter.
 *
 */
class SystemModel {
 public:
    explicit SystemModel(const Eigen::MatrixXd& noise);
    virtual ~SystemModel() = default;

    /**
     * @brief Set the control given
     *
     * @param control control given
     */
    virtual void setControl(const Eigen::VectorXd& control);

    /**
     * @brief Getter for the A matrix that multiplies the previous mean
     *
     * @return Eigen::MatrixXd A matrix
     */
    virtual Eigen::MatrixXd A() const;

    /**
     * @brief Getter for the B matrix that multiplies the control input
     *
     * @return Eigen::MatrixXd B matrix
     */
    virtual Eigen::MatrixXd B() const;


       /**
     * @brief Getter for the control of the system
     *
     * @return Eigen::VectorXd Control input
     */
    virtual Eigen::VectorXd control() const;      

    /**
     * @brief Noise of the system
     *
     * @return Eigen::MatrixXd Noise matrix
     */
    virtual Eigen::MatrixXd noise() const;

 protected:
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::VectorXd control_;
    Eigen::MatrixXd noise_;
    uint32_t dimensions_;
};

}  // namespace crf::math::kalmanfilter
