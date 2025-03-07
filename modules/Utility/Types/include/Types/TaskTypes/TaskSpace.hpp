/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Bartosz Sójka CERN BE/CEM/MRO 2023
 *
 *  ================================================================================================================
 */

#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <map>

namespace crf::utility::types {

/**
 * @ingroup group_task_space
 * @brief Class for enumerating direction in the tangent space to a task space.
*/
enum class TaskSpaceTangentDimension {
    /**
     * @brief Linear direction along x axis.
    */
    Vx,

    /**
     * @brief Linear direction along y axis.
    */
    Vy,

    /**
     * @brief Linear direction along z axis.
    */
    Vz,

    /**
     * @brief Angular direction around x axis.
    */
    Wx,

    /**
     * @brief Angular direction around Y axis.
    */
    Wy,

    /**
     * @brief Angular direction around Z axis.
    */
    Wz
};

/**
 * @ingroup group_task_types
 * @brief Type for representing potentially reduced task space.
 */
class TaskSpace {
 public:
    /**
     * Unreduced task space
    */
    TaskSpace();

    TaskSpace(const TaskSpace& other) = default;

    explicit TaskSpace(const std::map<TaskSpaceTangentDimension, bool>& taskSpace);

    explicit TaskSpace(const std::array<bool, 6>& taskSpace);

    /**
     * @throws Size -- Parameter 'coordinates' is checked if it has the size equal to 6, otherwise,
     * constructor throws a 'std::invalid_argument' exception.
    */
    explicit TaskSpace(const std::initializer_list<bool>& taskSpace);

    ~TaskSpace() = default;

    TaskSpace& operator=(const TaskSpace& other) = default;

    TaskSpace& operator=(const std::map<TaskSpaceTangentDimension, bool>& taskSpace);

    TaskSpace& operator=(const std::array<bool, 6>& taskSpace);

    /**
     * @throws Size -- Parameter 'taskSpace' is checked if it has the size equal to 6, otherwise,
     * operator throws a 'std::invalid_argument' exception.
    */
    TaskSpace& operator=(const std::initializer_list<bool>& taskSpace);

    /**
     * @brief Returns the number of dimensions in the task space.
    */
    size_t dimension() const;

    /**
     * @brief Returns the number of linear dimensions in the task space.
    */
    size_t linearDimension() const;

    /**
     * @brief Returns the number of angular dimensions in the task space.
    */
    size_t angularDimension() const;

    /**
     * @brief Returns a modyfied identity matrix with rows corresponding to unused dimensions
     * missing.
     */
    Eigen::Matrix<double, Eigen::Dynamic, 6> getNoRowsMatrix() const;

    /**
     * @brief Returns a modyfied identity matrix with rows corresponding to unused dimensions
     * filled with zeros.
     */
    Eigen::Matrix<double, 6, 6> getZeroRowsMatrix() const;

    /**
     * @brief Returns a modyfied matrix with rows corresponding to unused dimensions
     * filled with NaNs.
     */
    Eigen::Matrix<double, 6, 6> getNaNRowsMatrix() const;

    /**
     * @brief Returns an eigen vector with coordinates corresponding to used dimensions equal to 1
     * and to unused dimensions equal to 0.
     */
    Eigen::Vector<double, 6> getVector6d() const;

    /**
     * @brief Returns number of possible coordinates (in the task space or not). Always equal to 6.
     */
    std::size_t size() const;

    /**
     * @brief Non-Const version of the index operator, indexed by 'TaskSpaceTangentDimension' enum.
     */
    bool& operator[](const TaskSpaceTangentDimension& dimension);

    /**
     * @brief Const version of the index operator, indexed by 'TaskSpaceTangentDimension' enum.
     */
    bool operator[](const TaskSpaceTangentDimension& dimension) const;

    /**
     * @brief Non-Const version of the index operator, indexed by [0, 6].
     * @throw Range -- parameter 'index' is checked if it is within the range [0, 6], otherwise,
     * operator throws a 'std::out_of_range' exception.
     */
    bool& operator[](const size_t& index);

    /**
     * @brief Const version of the index operator, indexex by [0, 6].
     * @throw Range -- parameter 'index' is checked if it is within the range [0, 6], otherwise,
     * operator throws a 'std::out_of_range' exception.
     */
    bool operator[](const size_t& index) const;

 private:
    std::map<TaskSpaceTangentDimension, bool> taskSpace_;
};

/**
 * @brief Printable version of Vector6d
 */
std::ostream& operator<<(std::ostream& os, const TaskSpace& taskSpace);

}  // namespace crf::utility::types
