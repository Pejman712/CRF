/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jean Paul Sulca CERN BM-CEM-MRO 2023
 *         
 *
 *  ==================================================================================================
 */

#pragma once

#include <vector>
#include <string>

#include "MassSpringDamper/MSDRn/IMassSpringDamperRn.hpp"
#include "MassSpringDamper/MSDR1/IMassSpringDamperR1.hpp"

namespace crf::math::massspringdamper {

/**
 * @ingroup group_mass_spring_damper_rn
 * @brief This class is a template that takes an implementation of IMassSpringDamperR1, and
 *        increases the number of dimensions of the system, but decoupled between them.
 */
template<typename T, std::enable_if_t<std::is_base_of<IMassSpringDamperR1, T>::value>* = nullptr>
class MSDRnDecoupled : public IMassSpringDamperRn {
 public:
    /**
     * @brief Construct a MSDRnDecoupled object.
     *
     * @param dim Number of dimensions
     * @param Ts Time step for discrete signals
     */
    explicit MSDRnDecoupled(uint8_t dim, float Ts):
        dim_(dim), Ts_(Ts) {
        if (Ts <= 0) {
            throw std::invalid_argument("Time step can't be equal or less than zero");
        }
        for (uint8_t i = 0; i < dim; i++) {
            msdR1s_.push_back(T(Ts_));
        }
    }

    ~MSDRnDecoupled() = default;

    SignalRn calculate(const Eigen::MatrixXd &force, const Eigen::MatrixXd &m,
        const Eigen::MatrixXd &d, const Eigen::MatrixXd &k) override {
        if (!m.isDiagonal() || !d.isDiagonal() || !k.isDiagonal())
            throw std::invalid_argument("Matrix of parameters must be diagonal");
        if (m.isZero() && d.isZero() && k.isZero())
            throw std::invalid_argument("All the parameters can't be zero");
        if ((m.array() < 0).any() || (d.array() < 0).any() || (k.array() < 0).any())
            throw std::invalid_argument("No parameter can be negative");

        SignalRn signal;
        SignalR1 r1Signal;
        for (uint8_t i = 0; i < force.rows(); i++) {
            try {
                r1Signal = msdR1s_[i].calculate(force(i, i), m(i, i), d(i, i), k(i, i));
            } catch (std::invalid_argument &ex) {
                std::string msg = "A problem has ocurred inside the dimension " +
                    std::to_string(i) + ". Internal Error: " + ex.what();
                throw std::invalid_argument(msg);
            }

            signal.position.push_back(r1Signal.position);
            signal.velocity.push_back(r1Signal.velocity);
            signal.acceleration.push_back(r1Signal.acceleration);
        }
        return signal;
    }

 private:
    const uint8_t dim_;
    float Ts_;
    std::vector<T> msdR1s_;
};

}  // namespace crf::math::massspringdamper
