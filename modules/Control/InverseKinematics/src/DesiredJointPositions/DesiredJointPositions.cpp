/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include "InverseKinematics/DesiredJointPositions/DesiredJointPositions.hpp"

using crf::utility::types::JointPositions;

namespace crf::control::inversekinematics {

DesiredJointPositions::DesiredJointPositions(double rangeSinusoid, double cycleTime, double c):
    logger_("DesiredJointPositions"),
    rangeSinusoid_(rangeSinusoid),
    cycleTime_(cycleTime),
    c_(c) {
    logger_->debug("CTor");
    if (rangeSinusoid <= 0 || cycleTime <= 0 || c <= 0) {
        throw std::invalid_argument("Wrong value in one or some input arguments");
    }
    increasingSinusoid_.reset(new crf::math::geometricmethods::Sinusoid(0, 1, rangeSinusoid_,
        crf::math::geometricmethods::ComputationMethod::SetRange));
    decreasingSinusoid_.reset(new crf::math::geometricmethods::Sinusoid(1, 0, rangeSinusoid_,
        crf::math::geometricmethods::ComputationMethod::SetRange));
    inTransition_ = false;
    transtionEvaluationTime_ = -1.0;
    goToNextIteration_ = true;
    enabled_ = false;
    startTransition_ = false;
}

DesiredJointPositions::~DesiredJointPositions() {
    logger_->debug("DTor");
}

Eigen::MatrixXd DesiredJointPositions::getGradient(
    const crf::utility::types::JointPositions& q,
    const crf::utility::types::JointPositions& qAttr) {
    logger_->debug("getGradient");
    unsigned int qSize = q.size();
    if (qSize != qAttr.size()) {
        throw std::invalid_argument("Wrong size input");
    }
    Eigen::MatrixXd gradientPenalty(static_cast<int>(qSize), 1);
    transitionFactor_ = getTransitionFactor();
    for (unsigned int i = 0; i < qSize; i++) {
        if (std::isnan(qAttr[i])) {
            gradientPenalty(i, 0) = std::nan("");
        } else {
            gradientPenalty(i, 0) = (c_ * (qAttr[i] - q[i])) * transitionFactor_;
        }
    }
    return gradientPenalty;
}

std::vector<double> DesiredJointPositions::getTimeDerivative(
    const crf::utility::types::JointPositions& q,
    const crf::utility::types::JointVelocities& qd,
    const crf::utility::types::JointPositions& qAttr) {
    logger_->debug("getTimeDerivative");
    logger_->error("This method is still not implemented for this objective function");
    return std::vector<double>(0);
}

bool DesiredJointPositions::enable(bool state) {
    logger_->debug("setEnable");
    if (inTransition_ || startTransition_) {
        return false;
    }
    if (state != enabled_) {
        startTransition_ = true;
    }
    enabled_ = state;
    return true;
}

bool DesiredJointPositions::enable() const {
    logger_->debug("getEnable");
    return enabled_;
}

void DesiredJointPositions::goToNextIteration(const bool& next) {
    logger_->debug("goToNextIteration");
    goToNextIteration_ = next;
}

double DesiredJointPositions::getTransitionFactor() {
    logger_->debug("getTransitionFactor");
    if (goToNextIteration_ && inTransition_) {
        transtionEvaluationTime_ = transtionEvaluationTime_ + cycleTime_;
    }
    if (startTransition_) {
        startTransition_ = false;
        inTransition_ = true;
        transtionEvaluationTime_ = 0.0;
    }
    if (transtionEvaluationTime_ < 0.0 || transtionEvaluationTime_ > rangeSinusoid_) {
        inTransition_ = false;
        if (enabled_) {
            return 1.0;
        }
        // !enabled_
        return 0.0;
    }
    if (enabled_) {
        return increasingSinusoid_->evaluate(transtionEvaluationTime_, 0).value();
    }
    // !enabled_
    return decreasingSinusoid_->evaluate(transtionEvaluationTime_, 0).value();
}

bool DesiredJointPositions::setParam(std::string objFuncName, std::string value) {
    logger_->debug("setParam");
    if (std::stod(value) <= 0) {
        logger_->error("Wrong imput in value");
        return false;
    }
    if (enabled_ || inTransition_ || startTransition_) {
        logger_->error("Parameters cannot change if the objective function is enabled or "
            "in transition to be enabled/disabled");
        return false;
    }
    if (objFuncName == "c") {
        c_ = std::stod(value);
        return true;
    }
    logger_->error("Wrong imput in objFuncName");
    return false;
}

}  // namespace crf::control::inversekinematics
