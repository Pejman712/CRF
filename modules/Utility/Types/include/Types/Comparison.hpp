/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *         Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "Types/JointTypes/JointTypes.hpp"
#include "Types/TaskTypes/TaskTypes.hpp"

namespace crf::utility::types {

using crf::math::rotation::areAlmostEqual;

/**
 * @ingroup group_types_comparison
 * @brief Compares on each coordinate if parameters are the same
 * up to the parameter accuracy.
 */
bool areAlmostEqual(
    const VectorXd& vector1,
    const VectorXd& vector2,
    const double& accuracy = 1e-12);

/**
 * @ingroup group_types_comparison
 * @brief
 */
bool areAlmostEqual(
    const TaskPose& taskPose1,
    const TaskPose& taskPose2,
    const double& accuracy = 1e-12,
    const TaskSpace& mask = TaskSpace());

/**
 * @ingroup group_types_comparison
 * @brief Compares on each coordinate if parameters are the same
 * up to the parameter accuracy.
 */
bool areAlmostEqual(
    const Vector6d& vector1,
    const Vector6d& vector2,
    const double& accuracy = 1e-12,
    const TaskSpace& mask = TaskSpace());

/**
 * @ingroup group_types_comparison
 * @brief Compares if parameters 'taskSpace1' and 'taskSpace2' are equal.
*/
bool areEqual(const TaskSpace& taskSpace1, const TaskSpace& taskSpace2);

/**
 * @ingroup group_types_comparison
 * @brief Compares on each coordinate if parameter 'vector' is between the parameter
 * 'lowerBound' and the parameter 'upperbound'.
 */
bool isBetween(const VectorXd& lowerBound, const VectorXd& upperBound, const VectorXd& vector);

/**
 * @ingroup group_types_comparison
 * @brief Compares on each coordinate if parameter 'vector' is between the parameter
 * 'lowerBound' and the parameter 'upperbound'.
 */
bool isBetween(
    const Vector6d& lowerBound,
    const Vector6d& upperBound,
    const Vector6d& vector,
    const TaskSpace& mask = TaskSpace());

/**
 * @ingroup group_types_comparison
 * @brief Compares on each coordinate if the parameter 'vector1' is lesser than
 * the parameter 'vector2'.
 */
bool isLesser(const VectorXd& vector1, const VectorXd& vector2);

/**
 * @ingroup group_types_comparison
 * @brief Compares on each coordinate if the parameter 'vector1' is lesser than
 * the parameter 'vector2'.
 */
bool isLesser(
    const Vector6d& vector1,
    const Vector6d& vector2,
    const TaskSpace& mask = TaskSpace());

/**
 * @ingroup group_types_comparison
 * @brief Compares on each coordinate if the parameter 'vector1' is greater than
 * the parameter 'vector2'.
 */
bool isGreater(const VectorXd& vector1, const VectorXd& vector2);

/**
 * @ingroup group_types_comparison
 * @brief Compares on each coordinate if the parameter 'vector1' is greater than
 * the parameter 'vector2'.
 */
bool isGreater(
    const Vector6d& vector1,
    const Vector6d& vector2,
    const TaskSpace& mask = TaskSpace());

}  // namespace crf::utility::types
