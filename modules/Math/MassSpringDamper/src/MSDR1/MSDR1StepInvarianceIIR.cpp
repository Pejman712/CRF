/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#include <stdexcept>
#include <cmath>

#include "MassSpringDamper/MSDR1/MSDR1StepInvarianceIIR.hpp"

namespace crf::math::massspringdamper {

MSDR1StepInvarianceIIR::MSDR1StepInvarianceIIR(double Ts):
    MSDR1DirectForm2Abstract(Ts) {
}

MSDR1DirectForm2Abstract::Coefficients MSDR1StepInvarianceIIR::getCoefficients(
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

        // pole 1 = 0
        double p2 = (-d + root) / (2*m);  // pole 2
        double p3 = -(d + root) / (2*m);  // pole 3
        double normFact = p2*p3*(p2-p3);

        a[1] = -exp(p2*Ts_) - exp(p3*Ts_);
        a[2] = exp(p2*Ts_)*exp(p3*Ts_);
        if (discriminant > 0) {
            b[1] = (p3*exp(p2*Ts_) - p2*exp(p3*Ts_) + p2 - p3) / normFact;
            b[2] = (p2*exp((p2+p3)*Ts_) - p3*exp((p2+p3)*Ts_) - p2*exp(p2*Ts_) + p3*exp(p3*Ts_)) /
                normFact;
        } else {
            // Indetermination --> L'hopital
            b[1] = (exp(p2*Ts_)*(p2*Ts_-1) + 1) / pow(p2, 2);
            b[2] = exp(p2*Ts_)*(exp(p2*Ts_) - p2*Ts_ - 1) / pow(p2, 2);
        }
    } else {
        double sigma = -d / (2*m);
        double w = sqrt(-discriminant) / (2*m);

        a[1] = -2*exp(sigma*Ts_)*cos(w*Ts_);
        a[2] = exp(2*sigma*Ts_);
        b[1] = (sigma*exp(sigma*Ts_)/w * sin(w*Ts_) - exp(sigma*Ts_) * cos(w*Ts_) + 1) /
            (pow(sigma, 2) + pow(w, 2));
        b[2] = exp(sigma*Ts_)*(-sigma/w * sin(w*Ts_) - cos(w*Ts_) + exp(sigma*Ts_)) /
            (pow(sigma, 2) + pow(w, 2));
    }
    return {a, b};
}

}  // namespace crf::math::massspringdamper
