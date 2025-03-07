/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>
#include <optional>
#include <algorithm>
#include <iostream>

#include <Eigen/Dense>

#include "KalmanFilter/StateSpace/StateSpace.hpp"

namespace crf::math::kalmanfilter {

StateSpace::StateSpace(
    const Eigen::VectorXd& initialMean, const Eigen::MatrixXd& initialCovariance):
    ss_(initialMean),
    covariance_(initialCovariance) {
        if (initialCovariance.rows() != initialMean.size()) {
            throw std::invalid_argument(
                "StateSpace - CTor - Initial mean and covariance are not consistent in size");
        }
}

std::size_t StateSpace::dimensions() const {
    return ss_.size();
}

Eigen::VectorXd& StateSpace::getMean() {
    return ss_;
}

void StateSpace::setMean(const Eigen::VectorXd& stateSpace) {
    if (ss_.size() != stateSpace.size()) {
        throw std::runtime_error(
            "StateSpace - setMean - New mean can't have different size than previous old " +
            std::to_string(ss_.size()) + " vs new " + std::to_string(stateSpace.size()));
    }
    ss_ = stateSpace;
    //std::cout << "new mean " << ss_ << std::endl;
}

Eigen::MatrixXd StateSpace::getCovariance() const {
    return covariance_;
}

void StateSpace::setCovariance(const Eigen::MatrixXd& covariance) {
    if (covariance_.size() != covariance.size()) {
        throw std::runtime_error(
            "StateSpace - setCovariance - New covariance can't have different size than previous");
    }
    covariance_ = covariance;
    //std::cout << "new cova " << covariance_ << std::endl;
}

}  // namespace crf::math::kalmanfilter
