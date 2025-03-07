/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#include <stdexcept>
#include <cmath>

#include "MassSpringDamper/MSDR1/MSDR1BilinearTransformIIR.hpp"

namespace crf::math::massspringdamper {

MSDR1BilinearTransformIIR::MSDR1BilinearTransformIIR(double Ts):
    MSDR1DirectForm2Abstract(Ts) {
}

MSDR1DirectForm2Abstract::Coefficients MSDR1BilinearTransformIIR::getCoefficients(
    double m, double d, double k) {
    if (m == 0 && d == 0 && k == 0) {
        throw std::invalid_argument("All the parameters can't be zero");
    }
    if (m < 0 || d < 0 || k < 0) {
        throw std::invalid_argument("No parameter can be negative");
    }

    double b0 = pow(Ts_, 2)/(pow(Ts_, 2)*k+2*Ts_*d+4*m);
    double b1 = 2*b0;
    double b2 = b0;
    double a1 = (pow(Ts_, 2)*k-4*m)/(0.5*pow(Ts_, 2)*k+Ts_*d+2*m);
    double a2 = (pow(Ts_, 2)*k-2*d*Ts_+4*m)/(pow(Ts_,  2)*k+2*Ts_*d+4*m);

    // a0 will be the factor for normalization
    std::vector<double> a = {1, a1, a2};
    std::vector<double> b = {b0, b1, b2};

    return {a, b};
}

}  // namespace crf::math::massspringdamper
