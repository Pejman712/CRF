/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 * 
 *  ==================================================================================================
 */

#include <cmath>
#include "DigitalFilter/IIRFilter.hpp"
#include "MassSpringDamper/MSDR1/MSDR1DirectForm2Abstract.hpp"

namespace crf::math::massspringdamper {

MSDR1DirectForm2Abstract::MSDR1DirectForm2Abstract(double Ts):
    Ts_(Ts), u_({}), pos_(0), vel_(0) {
    if (Ts <= 0) {
        throw std::invalid_argument("Time step can't be equal or less than zero");
    }
}

SignalR1 MSDR1DirectForm2Abstract::calculate(double force, double m, double d,
    double k) {
    Coefficients coeffs = getCoefficients(m, d, k);

    crf::math::digitalfilter::DF2IIRFilter val = crf::math::digitalfilter::direct2FormFilter(
        force, coeffs.a, coeffs.b, u_);
    SignalR1 signal;
    signal.position = val.y;
    signal.velocity = (signal.position - pos_)/Ts_;
    signal.acceleration = (signal.velocity - vel_)/Ts_;

    u_ = val.u;
    pos_ = signal.position;
    vel_ = signal.velocity;
    return signal;
}

}  // namespace crf::math::massspringdamper
