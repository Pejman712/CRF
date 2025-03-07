
/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Dadi Hrannar Davidsson and Peji (hes mad) CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once
#include <Eigen/Dense>

#include "KalmanFilter/ObservationModel/ObservationModel.hpp"

namespace crf::math::kalmanfilter {

class SPSObservationModel : public ObservationModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit SPSObservationModel(const Eigen::MatrixXd& noise);
    SPSObservationModel() = delete;
    SPSObservationModel(const SPSObservationModel&) = delete;
    SPSObservationModel(SPSObservationModel&&) = delete;
    ~SPSObservationModel() override = default;
};

}  // namespace crf::math::kalmanfilter
