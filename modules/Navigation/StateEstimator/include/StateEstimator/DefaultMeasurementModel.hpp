/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#pragma once

#include "StateEstimator/kalman/LinearizedMeasurementModel.hpp"

namespace crf {
namespace algorithms {
namespace stateestimator {

template<int STATEVECTORSIZE, int MEASUREMENTVECTORSIZE>
class DefaultMeasurementModel: public Kalman::LinearizedMeasurementModel<
    Kalman::Vector<float, STATEVECTORSIZE> , Kalman::Vector<float, MEASUREMENTVECTORSIZE>,
    Kalman::StandardBase> {
 public:
    /**
    * This is the default measurement model
    */
    DefaultMeasurementModel() {
        this->H.setIdentity();
        this->V.setIdentity();
    }
    /**
     * Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    Kalman::Vector<float, MEASUREMENTVECTORSIZE> h(
        const Kalman::Vector<float, STATEVECTORSIZE>& x) const {
        Kalman::Vector<float, MEASUREMENTVECTORSIZE> measurement;

        for (int dim = 0; dim < MEASUREMENTVECTORSIZE; dim++) {
            measurement(dim) = x(dim);
        }
        return measurement;
    }
};

}  // namespace stateestimator
}  // namespace algorithms
}  // namespace crf
