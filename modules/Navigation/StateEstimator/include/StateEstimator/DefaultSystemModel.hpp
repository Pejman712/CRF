#pragma once
/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include "StateEstimator/kalman/LinearizedSystemModel.hpp"

namespace crf {
namespace algorithms {
namespace stateestimator {

template<int STATEVECTORSIZE>
class DefaultSystemModel : public Kalman::LinearizedSystemModel<
        Kalman::Vector<float, STATEVECTORSIZE>, Kalman::Vector<float, 0>, Kalman::StandardBase> {
 public:
    /**
    * This is the default system model
    */
    DefaultSystemModel() {
        this->F.setZero();
        this->W.setIdentity();
    }
    /**
     * Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    Kalman::Vector<float, STATEVECTORSIZE> f(const Kalman::Vector<float,
            STATEVECTORSIZE>& x, const Kalman::Vector<float, 0>& u) const {
        Kalman::Vector<float, STATEVECTORSIZE> x_ = x;
        // Return transitioned state vector
        return x_;
    }

 protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians(const Kalman::Vector<float, STATEVECTORSIZE>& x,
                         const Kalman::Vector<float, 0>& u ) {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setZero();
        this->W.setIdentity();
    }
};

}  // namespace stateestimator
}  // namespace algorithms
}  // namespace crf
