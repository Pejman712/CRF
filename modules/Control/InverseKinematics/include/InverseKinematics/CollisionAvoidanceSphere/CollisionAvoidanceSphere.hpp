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
 * @brief CollisionAvoidanceSphere Objective Function computes the gradient of the penalty according
 *        to the distance from the sphere set to be avoided.
 *        In this objective function, qAttr is not affecting at any moment.
 * 
 *        ATTENTION: The internal function calcMinDistRobot() changes depending on the robot.
 * 
 * @param rangeSinusoid is the range of time that the enabling/disabling transition takes
 * @param cycleTime is the time that takes each iteration
 * @param curveType can be quadratic (0) or exponential (1)
 * @param c is a tuning parameter of the penalty expression. It is used in both curve types:
 *        quadratic and exponential
 * @param p is a tuning parameter of the penalty expression only in the exponential curbe type
 * @param center is the center of the sphere object that is going to be avoided
 * @param radius is the radius of the sphere object that is going to be avoided
 * @param robot is a int value that defines the robot used. To use a different robot, the new
 *        corresponding mathematical expressions must be included in the calcMinDistRobot function.
 *            0 -> 2 DOF robot
 *            1 -> 3 DOF robot
 *            2 -> UR10e (6 DOF robot)
 *            3 -> TIMArm (9 DOF robot)
 */

class CollisionAvoidanceSphere : public IKinematicObjectiveFunction {
 public:
    explicit CollisionAvoidanceSphere(double rangeSinusoid, double cycleTime, int curveType,
        double c, double p, Eigen::Vector3d center, double radius, int robot);
    ~CollisionAvoidanceSphere() override;

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
    int curveType_;
    double c_;
    double p_;
    Eigen::Vector3d center_;
    double radius_;
    int robot_;
    const double dq_;
    std::unique_ptr<crf::math::geometricmethods::IGeometricMethods> increasingSinusoid_;
    std::unique_ptr<crf::math::geometricmethods::IGeometricMethods> decreasingSinusoid_;
    bool startTransition_;
    bool inTransition_;
    double transitionFactor_;
    double transtionEvaluationTime_;
    bool goToNextIteration_;
    bool enabled_;

    double calcMinDistLink(
        Eigen::VectorXd frame,
        Eigen::VectorXd nextFrame,
        Eigen::VectorXd obstacleCentre,
        double radius);
    Eigen::VectorXd calcMinDistRobot(
        crf::utility::types::JointPositions q,
        Eigen::VectorXd obstacleCentre);
};

}  // namespace crf::control::inversekinematics
