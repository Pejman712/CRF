
/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Dadi Hrannar Davidsson and Peji CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once
#include <vector>

#include <Eigen/Dense>

#include "KalmanFilter/SystemModel/SystemModel.hpp"

namespace crf::math::kalmanfilter {

class SPSSystemModel: public SystemModel{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit SPSSystemModel(const Eigen::MatrixXd& noise);
    
    SPSSystemModel() = delete;
    SPSSystemModel(const SPSSystemModel&) = delete;
    SPSSystemModel(SPSSystemModel&&) = delete;
    ~SPSSystemModel() override = default;
};

}
