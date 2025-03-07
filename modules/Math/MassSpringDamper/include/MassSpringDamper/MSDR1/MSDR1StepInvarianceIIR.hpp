/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include "MassSpringDamper/MSDR1/MSDR1DirectForm2Abstract.hpp"

namespace crf::math::massspringdamper {

/**
 * @ingroup group_sim_mass_spring_damper_r1
 * @brief The Mass-SpringDamper is modeled using the Step Invariance Method with Impulse
 *        Invariance Response in Direc Form 2 Filter.
 */
class MSDR1StepInvarianceIIR : public MSDR1DirectForm2Abstract {
 public:
    /**
     * @brief Construct a new MSDR1StepInvarianceIIR object.
     *
     * @param Ts Time step for discrete signals
     */
    MSDR1StepInvarianceIIR() = delete;
    explicit MSDR1StepInvarianceIIR(double Ts);
    ~MSDR1StepInvarianceIIR() override = default;

 private:
    Coefficients getCoefficients(double m, double d, double k) override;
};

}  // namespace crf::math::massspringdamper
