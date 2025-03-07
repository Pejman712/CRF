/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022
 *
 *  ================================================================================================================
*/

#pragma once

namespace crf::control::inversekinematics {

/**
 * @brief The flags are ordered in increasing order of importance
 *        The flags can include the others that are situated before. Example: If ResultFlags is
 *        workspaceViolation, also there is possible that computation time and/or the
 *        end-effector tolerance will be violated and/or the manipulability will be low. But if
 *        ResultFlags is endEffectorToleranceViolation, neither maxComputationTimeViolation
 *        neither workspaceViolation will happen
 */

enum class ResultFlags {
    /**
     * @brief Predefined value
     */
    notDefined = 0,
    /**
     * @brief The desired end-effector position has been achieved correctly
     */
    success = 1,
    /**
     * @brief Warn: The robot is close to a singularity
     */
    lowManipulability = 2,
    /**
     * @brief Error: The desired end-effector position has not been achieved inside the desired
     *        position tolerance. It migth be due to workspaceViolation or other reasons.
     */
    endEffectorToleranceViolation = 3,
    /**
     * @brief Error: It takes more time than the required
     */
    maxComputationTimeViolation = 4,
    /**
     * @brief Error: The end efector desired position is not reachable
     */
    workspaceViolation = 5
};

}  // namespace crf::control::inversekinematics
