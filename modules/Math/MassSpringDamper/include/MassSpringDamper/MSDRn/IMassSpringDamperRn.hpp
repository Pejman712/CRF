/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>

namespace crf::math::massspringdamper {

/**
 * @ingroup group_mass_spring_damper_rn
 * @brief 
 */
struct SignalRn {
    std::vector<double> position = std::vector<double>();
    std::vector<double> velocity = std::vector<double>();
    std::vector<double> acceleration = std::vector<double>();
};

/**
 * @ingroup group_mass_spring_damper_rn
 * @brief Interface for implementations of Mass Spring Damper system in n dimensions
 */
class IMassSpringDamperRn {
 public:
    virtual ~IMassSpringDamperRn() = default;

    /**
     * @brief Calculate the position, velocity and acceleration of the system at that moment,
     *        as a result of a force applied.
     *
     * @param force Force matrix applied as an input to the system.
     * @param m Mass matrix
     * @param d Damping coefficients matrix
     * @param k Spring constants matrix
     * @return Struct with the position, velocity and acceleration values. 
     */
    virtual SignalRn calculate(const Eigen::MatrixXd &force, const Eigen::MatrixXd &m,
        const Eigen::MatrixXd &d, const Eigen::MatrixXd &k) = 0;
};

}  // namespace crf::math::massspringdamper
