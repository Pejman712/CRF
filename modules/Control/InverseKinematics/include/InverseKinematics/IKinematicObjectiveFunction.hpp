/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include "Types/Types.hpp"

namespace crf::control::inversekinematics {

class IKinematicObjectiveFunction {
 public:
    virtual ~IKinematicObjectiveFunction() = default;

    /**
     * @brief Function that compute the derivative of the penalty with respect to the joint
     *        coordinates according to the objective function class that is using it
     * 
     * @param q is the set of joints position from which the penalty is computed and is going to
     *        be applied
     * @param qAttr is the attracting joints position used in the DesiredJointPositions Objective
     *        Function.
     *        In the rest of Objective Functions this parameter is not utilized, that is why it is
     *        predefined
     * @return the matrix of computed penalties, whose number of rows is the number of robot joints
     */
    virtual Eigen::MatrixXd getGradient(
        const crf::utility::types::JointPositions& q,
        const crf::utility::types::JointPositions& qAttr =
            crf::utility::types::JointPositions(1)) = 0;

    /**
     * @brief Function that compute the derivative of the penalty with respect to the time
     *        according to the objective function class that is using it
     * 
     * @param q is the set of joints position from which the penalty is computed and is going to
     *        be applied
     * @param qd
     * @param qAttr is the attracting joints position used in the DesiredJointPositions Objective
     *        Function.
     *        In the rest of Objective Functions this parameter is not utilized, that is why it is
     *        predefined
     * @return the vector of computed penalties, whose size is the number of robot joints
     */
    virtual std::vector<double> getTimeDerivative(
        const crf::utility::types::JointPositions& q,
        const crf::utility::types::JointVelocities& qd,
        const crf::utility::types::JointPositions& qAttr =
            crf::utility::types::JointPositions(1)) = 0;

    /**
     * @brief Function to set the state (enabled or disabled) of the objective function
     * 
     * @param state will be enabled if it is true (1) and will be disabled if it is false (0)
     * @return true if it is enabled correctly
     *         false in the case it couldn't be enabled. This happens when a transition is in process.
     */
    virtual bool enable(bool state) = 0;

    /**
     * @brief Function to get the state (enabled or disabled) of the objective function
     * 
     * @return true if the objective function is enabled
     *         false if the objective function is disabled
     */
    virtual bool enable() const = 0;

    /**
     * @brief In all the objective functions there is an internal counter to mesure the transition
     *        time. Before calling the functions getGradient and getTransitionFactor outside the
     *        OptCLIK and OptOLIK classes (if they are used) or, once one of them is called, before
     *        use the other or the same again, it is necessary setting to false the function
     *        goToNextIteration, and after, setting it to true to activate again the counter for
     *        the transition.
     *        Default: True.
     * 
     * @param if it is true (1) it goes to the next iteration automatically
     *        if it is false (0) it will remain in the same iteration
     */
    virtual void goToNextIteration(const bool& next) = 0;

    /**
     * @brief Function to get transition factor to multiply the penalty value during transitions
     * 
     * @return the transition factor
     */
    virtual double getTransitionFactor() = 0;

    /**
     * @brief Function to set the parameters of the objective function
     * 
     * @param objFuncName corresponds to a parameter name
     * @param value is the new value that the user wants to set for the parameter
     * @return true if the parameter is set correctly
     *         false in the case it couldn't be set.
     */
    virtual bool setParam(std::string objFuncName, std::string value) = 0;
};

}  // namespace crf::control::inversekinematics
