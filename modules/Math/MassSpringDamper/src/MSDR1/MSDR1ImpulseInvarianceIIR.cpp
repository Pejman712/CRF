/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#include <stdexcept>
#include <cmath>

#include "MassSpringDamper/MSDR1/MSDR1ImpulseInvarianceIIR.hpp"

namespace crf::math::massspringdamper {

MSDR1ImpulseInvarianceIIR::MSDR1ImpulseInvarianceIIR(double Ts):
    MSDR1DirectForm2Abstract(Ts) {
}

MSDR1DirectForm2Abstract::Coefficients MSDR1ImpulseInvarianceIIR::getCoefficients(
    double m, double d, double k) {
    if (m == 0 && d == 0 && k == 0) {
        throw std::invalid_argument("All the parameters can't be zero");
    }
    if (m < 0 || d < 0 || k < 0) {
        throw std::invalid_argument("No parameter can be negative");
    }

    std::vector<double> a = {1, 0, 0};  // Coefficients at the denominator
    std::vector<double> b = {0, 0, 0};  // Coefficients at the numerator

    // Coefficients that change depending on real or complex domain
    double discriminant = pow(d, 2) - 4*k*m;
    // Real domain -> equations applied directly
    if (discriminant >= 0) {
        double root = sqrt(discriminant);
        double p1 = (-d + root) / (2*m);  // pole 1
        double p2 = -(d + root) / (2*m);  // pole 2
        a[1] = -exp(p1*Ts_) - exp(p2*Ts_);
        a[2] = exp(p1*Ts_)*exp(p2*Ts_);
        if (discriminant > 0) {
            b[1] = Ts_ * (exp(p1*Ts_) - exp(p2*Ts_)) / (p1 - p2);
        } else {
            b[1] = pow(Ts_, 2)*exp(p2*Ts_);
        }

    // Complex domain -> transformation into sine and cosine
    } else {
        double sigma = -d / (2*m);
        double w = sqrt(-discriminant) / (2*m);

        a[1] = -2*exp(sigma*Ts_)*cos(w*Ts_);
        a[2] = exp(2*sigma*Ts_);
        b[1] = Ts_*exp(sigma)*sin(w*Ts_);
    }
    return {a, b};
}

}  // namespace crf::math::massspringdamper
