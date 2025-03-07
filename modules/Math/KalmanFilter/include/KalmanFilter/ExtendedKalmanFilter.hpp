/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#pragma once

#include <memory>

#include <Eigen/Dense>

#include "KalmanFilter/StateSpace/StateSpace.hpp"
#include "KalmanFilter/ObservationModelLinearized/ObservationModelLinearized.hpp"
#include "KalmanFilter/SystemModelLinearized/SystemModelLinearized.hpp"

namespace crf::math::kalmanfilter {

/**
 * @ingroup group_kalman_filter
 * @brief Implementation of the Extended Kalman Filter (EKF). Given a linearized system model and a
 * linearized observation model, the algortihm estimates the state space of the system.
 *
 */
class ExtendedKalmanFilter {
 public:
    ExtendedKalmanFilter() = default;
    ~ExtendedKalmanFilter() = default;

    /**
     * @brief Prediction stage of the algorithm
     *
     * @param systemModel Model of the system to use
     * @param space State space we are trying to estimate
     */
    void prediction(
        std::shared_ptr<SystemModelLinearized> systemModel,
        std::shared_ptr<StateSpace> space);

    /**
     * @brief Correction stage of the algorithm
     *
     * @param observationModel Model of the observations we'll receive
     * @param space State space we are trying to estimate
     */
    void correction(
        std::shared_ptr<ObservationModelLinearized> observationModel,
        std::shared_ptr<StateSpace> space);
};

}  // namespace crf::math::kalmanfilter
