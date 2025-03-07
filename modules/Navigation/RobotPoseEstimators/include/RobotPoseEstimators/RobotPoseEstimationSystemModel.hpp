/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#pragma once

#include <StateEstimator/kalman/LinearizedSystemModel.hpp>

#include <chrono>

namespace crf {
namespace applications {
namespace robotposeestimator {

class SystemControl : public Kalman::Vector<float, 0> {
 public:
    /**
    * This is the system control-input 
    */
    KALMAN_VECTOR(SystemControl, float, 0)
};

class SystemState : public Kalman::Vector<float, 7> {
 public:
    KALMAN_VECTOR(SystemState, float, 7)

    //! X-Velocity
    static constexpr size_t Xdot = 0;
    //! Y-Velocity
    static constexpr size_t Ydot = 1;
    //! Z-Velocity
    static constexpr size_t Zdot = 2;
    //! X-Acceleration
    static constexpr size_t Xdotdot = 3;
    //! Y-Acceleration
    static constexpr size_t Ydotdot = 4;
    //! Z-Acceleration
    static constexpr size_t Zdotdot = 5;
    //! change in yaw angle
    static constexpr size_t Yawdot = 6;

    float xdot()       const { return (*this)[ Xdot ]; }
    float ydot()       const { return (*this)[ Ydot ]; }
    float zdot()       const { return (*this)[ Zdot ]; }
    float xdotdot()       const { return (*this)[ Xdotdot ]; }
    float ydotdot()       const { return (*this)[ Ydotdot ]; }
    float zdotdot()       const { return (*this)[ Zdotdot ]; }
    float yawdot()       const { return (*this)[ Yawdot ]; }

    float& xdot()      { return (*this)[ Xdot ]; }
    float& ydot()      { return (*this)[ Ydot ]; }
    float& zdot()      { return (*this)[ Zdot ]; }
    float& xdotdot()      { return (*this)[ Xdotdot ]; }
    float& ydotdot()      { return (*this)[ Ydotdot ]; }
    float& zdotdot()      { return (*this)[ Zdotdot ]; }
    float& yawdot()       { return (*this)[ Yawdot ]; }
};

class SystemModel : public Kalman::LinearizedSystemModel<SystemState,
SystemControl, Kalman::StandardBase> {
 public:
    //! State type shortcut definition
    typedef SystemState S;
    typedef SystemControl C;
    std::chrono::high_resolution_clock::time_point previousMeasureTimestamp;
    bool firstMeasure = true;
    std::chrono::microseconds elapsedSeconds;

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
    S f(const S& x, const C& u) const {
            std::chrono::high_resolution_clock::time_point now =
            std::chrono::high_resolution_clock::now();
            S x_ = x;
            SystemModel* pointerThis = const_cast<SystemModel*>(this);
            if (!firstMeasure) {
                pointerThis->elapsedSeconds = std::chrono::duration_cast<
                std::chrono::microseconds>(now-previousMeasureTimestamp);
                //! Predicted state vector after transition
                float elapsedSeconds = static_cast<float>(
                    pointerThis->elapsedSeconds.count())*static_cast<float>(1e-6);
                x_.xdot() = x.xdot() + x.xdotdot()*elapsedSeconds;
                x_.ydot() = x.ydot() + x.ydotdot()*elapsedSeconds;
                x_.zdot() = x.zdot() + x.zdotdot()*elapsedSeconds;
                pointerThis->previousMeasureTimestamp = now;
            } else {
                pointerThis->firstMeasure = false;
                pointerThis->previousMeasureTimestamp = now;
                pointerThis->elapsedSeconds = std::chrono::microseconds(0);
            }
        // Return transitioned state vector
        return x_;
    }
};

}  // namespace robotposeestimator
}  // namespace applications
}  // namespace crf
