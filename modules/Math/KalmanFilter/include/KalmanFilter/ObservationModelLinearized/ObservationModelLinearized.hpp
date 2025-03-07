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
 * @brief Class to define the observation model of an Extended Kalman Filter (EKF). This class
 * implements the default observation model which is equivalent to an standard kalman filter. This
 * class can be inherited from to linearize your own observation model and evaluate it as preferred.
 *
 */
class ObservationModelLinearized {
 public:
    explicit ObservationModelLinearized(const Eigen::MatrixXd& noise);
    virtual ~ObservationModelLinearized() = default;

    /**
     * @brief This method evaluates the observation function h and it's jacobian H given a
     * predicted state.
     *
     * @param predictedState State predicted
     */
    virtual void evaluate(const Eigen::VectorXd& predictedState);

    /**
     * @brief Set the measurement taken
     *
     * @param measure Measurement taken
     */
    virtual void setMeasurement(const Eigen::VectorXd& measure);

    /**
     * @brief Getter for the function h evaluated
     *
     * @return Eigen::VectorXd Result of the h function
     */
    virtual Eigen::VectorXd h() const;

    /**
     * @brief Getter for the jacobian H evaluated
     *
     * @return Eigen::MatrixXd Result of the H jacobian
     */
    virtual Eigen::MatrixXd H() const;

    /**
     * @brief Last measurement taken
     *
     * @return Eigen::VectorXd Measurement taken
     */
    virtual Eigen::VectorXd measurement() const;

    /**
     * @brief Noise of the observation model. Normally related to the sensor noise.
     *
     * @return Eigen::MatrixXd Noise matrix
     */
    virtual Eigen::MatrixXd noise() const;

 protected:
    Eigen::VectorXd h_;
    Eigen::MatrixXd H_;
    Eigen::VectorXd measurement_;
    Eigen::MatrixXd noise_;
    uint32_t dimensions_;
};

}  // namespace crf::math::kalmanfilter
