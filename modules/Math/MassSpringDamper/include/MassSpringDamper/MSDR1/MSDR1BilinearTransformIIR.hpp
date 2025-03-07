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
 * @ingroup group_btm_mass_spring_damper_r1
 * @brief The Mass-SpringDamper is modeled using the Bilinear Transform Method with Impulse
 *        Invariance Response in Direc Form 2 Filter.
 */
class MSDR1BilinearTransformIIR : public MSDR1DirectForm2Abstract {
 public:
    /**
     * @brief Construct a new MSDR1BilinearTransformIIR object.
     *
     * @param Ts Time step for discrete signals
     */
    MSDR1BilinearTransformIIR() = delete;
    explicit MSDR1BilinearTransformIIR(double Ts);
    ~MSDR1BilinearTransformIIR() override = default;

 private:
    Coefficients getCoefficients(double m, double d, double k) override;
};

}  // namespace crf::math::massspringdamper
