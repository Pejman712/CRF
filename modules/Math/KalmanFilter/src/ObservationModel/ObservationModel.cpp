/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>

#include "KalmanFilter/ObservationModel/ObservationModel.hpp"

namespace crf::math::kalmanfilter {

ObservationModel::ObservationModel(const Eigen::MatrixXd& noise) {
    if (noise.rows() != noise.cols()) {
        throw std::invalid_argument("ObservationModel - CTor - Input noise matrix is not squared");
    }
    dimensions_ = noise.rows();
    noise_ = noise;

    // Defualt observation model
    C_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    measurement_ = Eigen::VectorXd(dimensions_);
}

void ObservationModel::setMeasurement(const Eigen::VectorXd& measure) {
    measurement_ = measure;
}

Eigen::MatrixXd ObservationModel::C() const {
    return C_;
}

Eigen::VectorXd ObservationModel::measurement() const {
    return measurement_;
}

Eigen::MatrixXd ObservationModel::noise() const {
    return noise_;
}

}  // namespace crf::math::kalmanfilter
