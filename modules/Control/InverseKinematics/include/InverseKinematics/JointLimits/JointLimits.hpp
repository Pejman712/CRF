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

#include "EventLogger/EventLogger.hpp"
#include "InverseKinematics/IKinematicObjectiveFunction.hpp"
#include "GeometricMethods/IGeometricMethods.hpp"
#include "GeometricMethods/Sinusoid/Sinusoid.hpp"

using crf::utility::types::JointPositions;

namespace crf::control::inversekinematics {

/**
 * @brief Joint Limits Objective Function computes the gradient of the penalty according to the
 *        setted minimum and maximum limits of the joints.
 *        In this function, qAttr is not affecting at any moment.
 * 
 * @param rangeSinusoid is the range of time that the enabling/disabling transition takes
 * @param cycleTime is the time that takes each iteration
 * @param c is a tuning parameter in the exponent of the penalty expression
 * @param p is a tuning parameter in the base of the penalty expression
 * @param minLimits is the minimum limit of the joints position
 * @param maxLimits is the maximum limit of the joints position
 */

class JointLimits : public IKinematicObjectiveFunction {
 public:
    JointLimits(double rangeSinusoid, double cycleTime, double c, double p,
        JointPositions minLimits, JointPositions maxLimits);
    ~JointLimits() override;

    Eigen::MatrixXd getGradient(
        const JointPositions& q,
        const JointPositions& qAttr = JointPositions(1)) override;
    std::vector<double> getTimeDerivative(
        const crf::utility::types::JointPositions& q,
        const crf::utility::types::JointVelocities& qd,
        const crf::utility::types::JointPositions& qAttr = JointPositions(1)) override;
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
    double p_;
    JointPositions qMinLimit_;
    JointPositions qMaxLimit_;
    unsigned int qMinLimitsSize_;
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
