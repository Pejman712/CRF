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
#include "KalmanFilter/ObservationModel/ObservationModel.hpp"
#include "KalmanFilter/SystemModel/SystemModel.hpp"

namespace crf::math::kalmanfilter {

/**
 * @ingroup group_kalman_filter
 * @brief Implementation of the Kalman Filter (KF). Given a linear system model and a linear
 * observation model, the algortihm estimates the state space of the system.
 *
 */
class KalmanFilter {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KalmanFilter() = default;
    ~KalmanFilter() = default;

    /**
     * @brief Prediction stage of the algorithm
     *
     * @param systemModel Model of the system to use
     * @param space State space we are trying to estimate
     */
    void prediction(
        std::shared_ptr<SystemModel> systemModel,
        std::shared_ptr<StateSpace> space);

    /**
     * @brief Correction stage of the algorithm
     *
     * @param observationModel Model of the observations we'll receive
     * @param space State space we are trying to estimate
     */
    void correction(
        std::shared_ptr<ObservationModel> observationModel,
        std::shared_ptr<StateSpace> space);
};

}  // namespace crf::math::kalmanfilter
