/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#include "InverseKinematics/ResultsIK.hpp"

namespace crf::control::inversekinematics {

ResultsIK::ResultsIK():
    zDesired_(TaskPose()),
    zdDesired_(TaskVelocity()),
    zddDesired_(TaskAcceleration()),
    qResult_(JointPositions(1)),
    qdResult_(JointVelocities(1)),
    qddResult_(JointAccelerations(1)),
    flag_(crf::control::inversekinematics::ResultFlags::notDefined),
    zError_(std::vector<double>()),
    kinematicManipulability_(),
    penaltyGradients_(Eigen::MatrixXd()) {
}

ResultsIK::ResultsIK(const ResultsIK& input):
    zDesired_(input.zDesired_),
    zdDesired_(input.zdDesired_),
    zddDesired_(input.zddDesired_),
    qResult_(input.qResult_),
    qdResult_(input.qdResult_),
    qddResult_(input.qddResult_),
    flag_(input.flag_),
    zError_(input.zError_),
    kinematicManipulability_(input.kinematicManipulability_),
    penaltyGradients_(input.penaltyGradients_) {
}

ResultsIK::~ResultsIK() {
}

ResultsIK& ResultsIK::operator=(const ResultsIK& other) {
    zDesired_ = other.zDesired_;
    zdDesired_ = other.zdDesired_;
    zddDesired_ = other.zddDesired_;
    qResult_ = other.qResult_;
    qdResult_ = other.qdResult_;
    qddResult_ = other.qddResult_;
    flag_ = other.flag_;
    zError_ = other.zError_;
    kinematicManipulability_ = other.kinematicManipulability_;
    penaltyGradients_ = other.penaltyGradients_;
    return *this;
}

void ResultsIK::zDesired(TaskPose input) {
    zDesired_ = input;
}

TaskPose ResultsIK::zDesired() const {
    return zDesired_;
}

TaskVelocity ResultsIK::zdDesired() const {
    return zdDesired_;
}

void ResultsIK::zdDesired(TaskVelocity input) {
    zdDesired_ = input;
}

TaskAcceleration ResultsIK::zddDesired() const {
    return zddDesired_;
}

void ResultsIK::zddDesired(TaskAcceleration input) {
    zddDesired_ = input;
}

void ResultsIK::qResult(JointPositions input) {
    qResult_ = input;
}

JointPositions ResultsIK::qResult() const {
    return qResult_;
}

JointVelocities ResultsIK::qdResult() const {
    return qdResult_;
}

void ResultsIK::qdResult(JointVelocities input) {
    qdResult_ = input;
}

JointAccelerations ResultsIK::qddResult() const {
    return qddResult_;
}

void ResultsIK::qddResult(JointAccelerations input) {
    qddResult_ = input;
}

void ResultsIK::flag(ResultFlags input) {
    flag_ = input;
}

ResultFlags ResultsIK::flag() const {
    return flag_;
}

void ResultsIK::zError(std::vector<double> input) {
    zError_ = input;
}

std::vector<double> ResultsIK::zError() const {
    return zError_;
}

void ResultsIK::kinematicManipulability(double input) {
    kinematicManipulability_ = input;
}

double ResultsIK::kinematicManipulability() const {
    return kinematicManipulability_;
}

void ResultsIK::penaltyGradients(Eigen::MatrixXd input) {
    penaltyGradients_ = input;
}

Eigen::MatrixXd ResultsIK::penaltyGradients() const {
    return penaltyGradients_;
}

}  // namespace crf::control::inversekinematics
