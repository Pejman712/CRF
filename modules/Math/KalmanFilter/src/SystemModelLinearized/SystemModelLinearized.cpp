/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>

#include "KalmanFilter/SystemModelLinearized/SystemModelLinearized.hpp"

namespace crf::math::kalmanfilter {

SystemModelLinearized::SystemModelLinearized(const Eigen::MatrixXd& noise) {
    if (noise.rows() != noise.cols()) {
        throw std::invalid_argument("SystemModel - CTor - Input noise matrix is not squared");
    }
    dimensions_ = noise.rows();
    noise_ = noise;

    // Default System
    G_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    g_ = Eigen::VectorXd::Zero(dimensions_);
}

void SystemModelLinearized::evaluate(const Eigen::VectorXd& mean, const Eigen::VectorXd& control) {
    g_ = mean;
}

Eigen::VectorXd SystemModelLinearized::g() const {
    return g_;
}

Eigen::MatrixXd SystemModelLinearized::G() const {
    return G_;
}

Eigen::MatrixXd SystemModelLinearized::noise() const {
    return noise_;
}

}  // namespace crf::math::kalmanfilter
