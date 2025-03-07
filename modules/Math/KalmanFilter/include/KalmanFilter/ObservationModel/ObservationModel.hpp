/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <Eigen/Dense>

namespace crf::math::kalmanfilter {

/**
 * @ingroup group_kalman_filter
 * @brief Class to define the observation model of a Kalman Filter. This class implements the
 * default observation model of observing the variable we are trying to estimate directly.
 *
 */
class ObservationModel {
 public:
    explicit ObservationModel(const Eigen::MatrixXd& noise);
    virtual ~ObservationModel() = default;

    /**
     * @brief Set the measurement taken
     *
     * @param measure Measure taken
     */
    virtual void setMeasurement(const Eigen::VectorXd& measure);

    /**
     * @brief Getter for the matrix of observation for the Kalman Filter
     *
     * @return Eigen::MatrixXd Observation matrix
     */
    virtual Eigen::MatrixXd C() const;

    /**
     * @brief Getter for the measurement registered inside the model
     *
     * @return Eigen::VectorXd Last measurement registered
     */
    virtual Eigen::VectorXd measurement() const;

    /**
     * @brief Noise matrix of the observation model. This can be normally related to the noise of
     * the sensor used.
     *
     * @return Eigen::MatrixXd Noise matrix
     */
    virtual Eigen::MatrixXd noise() const;

 protected:
    Eigen::MatrixXd C_;
    Eigen::MatrixXd noise_;
    Eigen::VectorXd measurement_;
    uint32_t dimensions_;
};

}  // namespace crf::math::kalmanfilter
