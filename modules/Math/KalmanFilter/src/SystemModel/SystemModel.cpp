/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>

#include <Eigen/Dense>

#include "KalmanFilter/SystemModel/SystemModel.hpp"

namespace crf::math::kalmanfilter {

SystemModel::SystemModel(const Eigen::MatrixXd& noise) {
    if (noise.rows() != noise.cols()) {
        throw std::invalid_argument("SystemModel - CTor - Input noise matrix is not squared");
    }
    dimensions_ = noise.rows();
    noise_ = noise;

    // Default system model
    A_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    B_ = Eigen::MatrixXd::Identity(dimensions_, dimensions_);
    control_ = Eigen::VectorXd::Zero(dimensions_);
}

void SystemModel::setControl(const Eigen::VectorXd& control) {
    control_ = control;
}

Eigen::VectorXd SystemModel::control() const {
    return control_;
}

Eigen::MatrixXd SystemModel::A() const {
    return A_;
}

Eigen::MatrixXd SystemModel::B() const {
    return B_;
}

Eigen::MatrixXd SystemModel::noise() const {
    return noise_;
}

}  // namespace crf::math::kalmanfilter
