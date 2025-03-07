/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2024
 *
 *  ================================================================================================================
 */

#include <memory>

#include <Eigen/Dense>

#include "KalmanFilter/ExtendedKalmanFilter.hpp"

namespace crf::math::kalmanfilter {

void ExtendedKalmanFilter::prediction(
    std::shared_ptr<SystemModelLinearized> sys, std::shared_ptr<StateSpace> space) {
    Eigen::VectorXd predMean = sys->g();
    Eigen::MatrixXd predCova = sys->G()*space->getCovariance()*sys->G().transpose() + sys->noise();
    space->setMean(predMean);
    space->setCovariance(predCova);
}

void ExtendedKalmanFilter::correction(
    std::shared_ptr<ObservationModelLinearized> obs, std::shared_ptr<StateSpace> space) {
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(space->dimensions(), space->dimensions());
    Eigen::MatrixXd innovCov = obs->H()*space->getCovariance()*obs->H().transpose() + obs->noise();
    Eigen::MatrixXd kalmanGain = space->getCovariance()*obs->H().transpose()*innovCov.inverse();
    Eigen::VectorXd corrMean = space->getMean() + kalmanGain*(obs->measurement() - obs->h());
    Eigen::MatrixXd corrCova = (identity - kalmanGain*obs->H())*space->getCovariance();
    space->setMean(corrMean);
    space->setCovariance(corrCova);
}

}  // namespace crf::math::kalmanfilter
