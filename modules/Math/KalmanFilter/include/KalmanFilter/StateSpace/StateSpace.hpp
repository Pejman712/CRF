/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>
#include <optional>
#include <map>

#include <Eigen/Dense>

namespace crf::math::kalmanfilter {

/**
 * @ingroup group_kalman_filter
 * @brief Class to implement the correspondent state space of the Kalman Filter. This
 * implementation only sets the intial values and provides getter and setters. The objective of
 * this class is to be inherited from and for the user to implement extra functions to interact
 * with their state space in a strongly typed way rather than through a vector directly.
 *
 */
class StateSpace {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateSpace(const Eigen::VectorXd& initialMean, const Eigen::MatrixXd& initialCovariance);
    virtual ~StateSpace() = default;

    /**
     * @brief Dimensions of the state space
     *
     * @return int Number of dimensions
     */
    virtual std::size_t dimensions() const;

    /**
     * @brief Get the mean of the state space
     *
     * @return Eigen::VectorXd Mean of the state space
     */
    virtual Eigen::VectorXd& getMean();

    /**
     * @brief Set the mean of the state space
     *
     * @param stateSpace Mean to set on the state space
     */
    virtual void setMean(const Eigen::VectorXd& stateSpace);

    /**
     * @brief Get the covariance matrix of the state space
     *
     * @return Eigen::MatrixXd Covariance matrix
     */
    virtual Eigen::MatrixXd getCovariance() const;

    /**
     * @brief Set the Covariance matrix of the state space
     *
     * @param covariance Covariance matrix
     */
    virtual void setCovariance(const Eigen::MatrixXd& covariance);


 protected:
    Eigen::VectorXd ss_;
    Eigen::MatrixXd covariance_;
};

}  // namespace crf::math::kalmanfilter
