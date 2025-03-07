/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>

#include "MassSpringDamper/MSDR1/IMassSpringDamperR1.hpp"

namespace crf::math::massspringdamper {

/**
 * @ingroup group_df2_mass_spring_damper_r1
 * @brief  This class models a Mass-Spirng-Damper System using a Direct Form 2 Filter. To use this
 *         class the private function getCoefficients must be defined.
 */
class MSDR1DirectForm2Abstract : public IMassSpringDamperR1 {
 public:
    /**
     * @brief Construct a MSDR1DirectForm2Abstract object.
     * 
     * @param Ts Time step for discrete signals.
     */
    explicit MSDR1DirectForm2Abstract(double Ts);
    MSDR1DirectForm2Abstract() = delete;
    ~MSDR1DirectForm2Abstract() override = default;

    SignalR1 calculate(double force, double m, double d, double k) override;

 protected:
    double Ts_;
    std::vector<double> u_;
    double pos_, vel_;

    struct Coefficients {
        std::vector<double> a;
        std::vector<double> b;
    };

    /**
     * @brief Calculates the coefficients of the direct from 2 filter.
     * 
     * @param m Mass.
     * @param d Damping coefficient.
     * @param k Spring constant.
     * @return The filter coefficients.
     */
    virtual Coefficients getCoefficients(double m, double d, double k) = 0;
};

}  // namespace crf::math::massspringdamper
