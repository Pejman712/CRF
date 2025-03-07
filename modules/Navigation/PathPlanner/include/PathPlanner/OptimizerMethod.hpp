/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2019
 *         Jorge Playan Garai CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */
#pragma once

namespace crf::navigation::pathplanner {

/**
 * @ingroup group_path_planner
 * @brief Enumeration of optimization methods for trajectory planning.
 *
 * The OptimizerMethod enum defines different optimization objectives for trajectory planning algorithms.
 *
 */
enum class OptimizerMethod {
    /**
     * @brief Default value indicating that the optimizer method is not defined.
     *
     */
    NotDefined = 0,

    /**
     * @brief Optimize the objective under the presence of state constraints.
     *
     */
    ConstraintObjective = 1,

    /**
     * @brief Optimize the duration or timing of control inputs.
     *
     */
    ControlDuration = 2,

    /**
     * @brief Minimize the mechanical work associated with the control inputs.
     *
     */
    MechanicalWork = 3,

    /**
     * @brief Minimize the maximum value of the objective over the trajectory.
     *
     */
    MinimaxObjective = 4,

    /**
     * @brief Maximize the minimum clearance to obstacles along the trajectory.
     *
     */
    MaximizeMinClearance = 5,

    /**
     * @brief Minimize the time taken to reach the goal.
     *
     */
    MinimizeArrivalTime = 6,

    /**
     * @brief Support for multiple optimization objectives.
     *
     */
    MultiOptimization = 7,

    /**
     * @brief An optimization objective which corresponds to optimizing path length.
     *
     */
    PathLength = 8,

    /**
     * @brief Minimize the integral of a cost function associated with the state.
     *
     */
    StateCostIntegral = 9,

    /**
     * @brief Specific optimization criterion related to a vector field (VF).
     *
     */
    VFUpstreamCriterion = 10,
};

}  // namespace crf::navigation::pathplanner
