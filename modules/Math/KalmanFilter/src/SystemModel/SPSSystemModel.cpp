/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>

#include <Eigen/Dense>

#include "KalmanFilter/SPSmodel/SPSSystemModel.hpp"

namespace crf::math::kalmanfilter {

SPSSystemModel::SPSSystemModel(const Eigen::MatrixXd& noise):
    SystemModel(noise)  {
    double dt = 0.1;

    A_ = Eigen::MatrixXd::Identity(3, 3);
    B_ = Eigen::MatrixXd::Zero(3, 6);
    B_(0, 0) = dt;
    B_(0, 3) = 0.5 * dt * dt;
    //B_(1, 1) = dt;
    B_(1, 4) = 0.5 * dt * dt;
    B_(2, 2) = dt;
    B_(2, 5) = 1;

    control_ = Eigen::VectorXd::Zero(6, 1);
}

}  // namespace crf::math::kalmanfilter
