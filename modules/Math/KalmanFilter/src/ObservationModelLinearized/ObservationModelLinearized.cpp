/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>

#include <Eigen/Dense>

#include "KalmanFilter/ObservationModelLinearized/ObservationModelLinearized.hpp"

namespace crf::math::kalmanfilter {

ObservationModelLinearized::ObservationModelLinearized(const Eigen::MatrixXd& noise) {
    if (noise.rows() != noise.cols()) {
        throw std::invalid_argument(
            "ObservationModel - CTor - Input noise matrix is not squared");
    }
    dimensions_ = noise.rows();
    noise_ = noise;

    // Defualt observation model
    H_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    h_ = Eigen::VectorXd(dimensions_);
    measurement_ = Eigen::VectorXd(dimensions_);
}

void ObservationModelLinearized::evaluate(const Eigen::VectorXd& predStateSpace) {
    h_ = predStateSpace;
}

void ObservationModelLinearized::setMeasurement(const Eigen::VectorXd& measure) {
    measurement_ = measure;
}

Eigen::VectorXd ObservationModelLinearized::h() const {
    return h_;
}

Eigen::MatrixXd ObservationModelLinearized::H() const {
    return H_;
}

Eigen::VectorXd ObservationModelLinearized::measurement() const {
    return measurement_;
}

Eigen::MatrixXd ObservationModelLinearized::noise() const {
    return noise_;
}

}  // namespace crf::math::kalmanfilter
