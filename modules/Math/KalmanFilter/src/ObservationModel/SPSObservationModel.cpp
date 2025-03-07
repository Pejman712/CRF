/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#include <vector>

#include "KalmanFilter/SPSmodel/SPSObservationModel.hpp"

namespace crf::math::kalmanfilter {

SPSObservationModel::SPSObservationModel(const Eigen::MatrixXd& noise):
    ObservationModel(noise) {

    C_ = Eigen::MatrixXd::Identity(3, 3);
    measurement_ = Eigen::VectorXd(3);
}

}  // namespace crf::math::kalmanfilter
