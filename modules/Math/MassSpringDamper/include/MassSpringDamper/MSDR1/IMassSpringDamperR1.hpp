/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

namespace crf::math::massspringdamper {

/**
 * @ingroup group_mass_spring_damper_r1
 * @brief Structure with all the values that a model of a mass spring damper system can calculate.
 */
struct SignalR1 {
    double position = 0;
    double velocity = 0;
    double acceleration = 0;
};

/**
 * @ingroup group_mass_spring_damper_r1
 * @brief Interface for implementations of Mass Spring Damper system in one dimension
 */
class IMassSpringDamperR1 {
 public:
    virtual ~IMassSpringDamperR1() = default;

    /**
     * @brief Calculate the position, velocity and acceleration of the system at that moment,
     *        as a result of a force applied.
     *
     * @param force Force applied as an input to the system.
     * @param m Mass
     * @param d Damping coefficient
     * @param k Spring constant
     * @return Struct with the position, velocity and acceleration values. 
     */
    virtual SignalR1 calculate(double force, double m, double d, double k) = 0;
};

}  // namespace crf::math::massspringdamper
