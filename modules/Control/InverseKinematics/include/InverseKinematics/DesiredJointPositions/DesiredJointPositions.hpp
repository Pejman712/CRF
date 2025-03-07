/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "EventLogger/EventLogger.hpp"
#include "InverseKinematics/IKinematicObjectiveFunction.hpp"
#include "GeometricMethods/IGeometricMethods.hpp"
#include "GeometricMethods/Sinusoid/Sinusoid.hpp"

using crf::utility::types::JointPositions;

namespace crf::control::inversekinematics {

/**
 * @brief Desired Joint Position Objective Function computes the gradient of the penalty according
 *        to the setted attractor (or desired) joints position, qAttr.
 *        qAttr must have the same size as q and qd. qAttr can contain std::nan("") in the elements
 *        that doesn't have a desired position.
 * 
 * @param rangeSinusoid is the range of time that the enabling/disabling transition takes
 * @param cycleTime is the time that takes each iteration
 * @param c is a tuning parameter of the penalty expression
 */

class DesiredJointPositions : public IKinematicObjectiveFunction {
 public:
    explicit DesiredJointPositions(double rangeSinusoid, double cycleTime, double c);
    ~DesiredJointPositions() override;

    Eigen::MatrixXd getGradient(
        const JointPositions& q,
        const JointPositions& qAttr) override;
    std::vector<double> getTimeDerivative(
        const crf::utility::types::JointPositions& q,
        const crf::utility::types::JointVelocities& qd,
        const crf::utility::types::JointPositions& qAttr) override;
    bool enable(bool state) override;
    bool enable() const override;
    void goToNextIteration(const bool& next) override;
    double getTransitionFactor() override;
    bool setParam(std::string objFuncName, std::string value) override;

 private:
    utility::logger::EventLogger logger_;
    double rangeSinusoid_;
    double cycleTime_;  // It must match with the time introduced in OptCLIK CTor
    double c_;
    std::unique_ptr<crf::math::geometricmethods::IGeometricMethods> increasingSinusoid_;
    std::unique_ptr<crf::math::geometricmethods::IGeometricMethods> decreasingSinusoid_;
    bool startTransition_;
    bool inTransition_;
    double transitionFactor_;
    double transtionEvaluationTime_;
    bool goToNextIteration_;
    bool enabled_;
};

}  // namespace crf::control::inversekinematics
