/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include "KalmanFilter/KalmanFilter.hpp"
 
namespace crf::math::kalmanfilter {

void KalmanFilter::prediction(
    std::shared_ptr<SystemModel> sys, std::shared_ptr<StateSpace> space) {
    Eigen::VectorXd predMean = sys->A()*space->getMean() + sys->B()*sys->control();
    Eigen::MatrixXd predCova = sys->A()*space->getCovariance()*sys->A().transpose() + sys->noise();

    //std::cout << "pred mean " << predMean << std::endl;
    //std::cout << "pred cova " << predCova << std::endl;
    space->setMean(predMean);
    space->setCovariance(predCova);
}

void KalmanFilter::correction(
    std::shared_ptr<ObservationModel> obs, std::shared_ptr<StateSpace> space) {
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(space->dimensions(), space->dimensions());
    Eigen::MatrixXd innovCov = obs->C()*space->getCovariance()*obs->C().transpose() + obs->noise();
    Eigen::MatrixXd kalmanGain = space->getCovariance()*obs->C().transpose()*innovCov.inverse();
    Eigen::VectorXd corrMean = space->getMean() + kalmanGain*(obs->measurement() - obs->C()*space->getMean());  // NOLINT
    Eigen::MatrixXd corrCova = (identity - kalmanGain*obs->C())*space->getCovariance();

    //std::cout << "corr mean " << corrMean << std::endl;
    //std::cout << "corr cova " << corrCova << std::endl;
    space->setMean(corrMean);
    space->setCovariance(corrCova);

}

}  // namespace crf::math::kalmanfilter
